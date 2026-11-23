// SPDX-License-Identifier: Apache-2.0
// Watch the host's foreground Docker client and the detector process group.
#define _GNU_SOURCE
#include <errno.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/signalfd.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>

static int same_owner(pid_t pid, const char *ticks)
{
  char path[64], stat[4096], *field;
  snprintf(path, sizeof(path), "/proc/%d/stat", pid);
  FILE *file = fopen(path, "r");
  if (!file) return 0;
  const int read = fgets(stat, sizeof(stat), file) != NULL;
  fclose(file);
  if (!read || !(field = strrchr(stat, ')'))) return 0;
  char *save = NULL;
  field = strtok_r(field + 1, " ", &save);
  for (int i = 3; i < 22 && field; ++i) field = strtok_r(NULL, " ", &save);
  return field && strcmp(field, ticks) == 0;
}

int main(int argc, char **argv)
{
  if (argc < 4) return 2;
  const pid_t owner = (pid_t)atoi(argv[1]);
  const int owner_fd = syscall(SYS_pidfd_open, owner, 0);
  if (owner_fd < 0 || !same_owner(owner, argv[2])) {
    fputs("AivudaOS owner exited before detector startup\n", stderr);
    return 1;
  }
  sigset_t mask, previous;
  sigemptyset(&mask);
  sigaddset(&mask, SIGINT);
  sigaddset(&mask, SIGTERM);
  if (sigprocmask(SIG_BLOCK, &mask, &previous) != 0) return 1;
  const int signals = signalfd(-1, &mask, SFD_CLOEXEC);
  if (signals < 0) return 1;
  const pid_t child = fork();
  if (child < 0) return 1;
  if (child == 0) {
    if (setpgid(0, 0) < 0 || sigprocmask(SIG_SETMASK, &previous, NULL) < 0) _exit(1);
    execvp(argv[3], argv + 3);
    perror(argv[3]);
    _exit(127);
  }
  setpgid(child, child);
  const int child_fd = syscall(SYS_pidfd_open, child, 0);
  struct pollfd fds[] = {
    {.fd = owner_fd, .events = POLLIN},
    {.fd = child_fd, .events = POLLIN},
    {.fd = signals, .events = POLLIN}
  };
  int ready = -1;
  if (child_fd >= 0) {
    do { ready = poll(fds, 3, -1); } while (ready < 0 && errno == EINTR);
  }
  const int stopping = fds[0].revents || fds[2].revents;
  kill(-child, SIGINT);
  if (ready >= 0 && !fds[1].revents) {
    do { ready = poll(&fds[1], 1, 5000); } while (ready < 0 && errno == EINTR);
  }
  kill(-child, SIGKILL);
  int status = 0;
  if (waitpid(child, &status, 0) < 0 || ready < 0) return 1;
  if (stopping) return 0;
  return WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
}
