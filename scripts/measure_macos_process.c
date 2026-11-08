// Build: cc -O2 scripts/measure_macos_process.c -o build/measure_macos_process
// Usage: measure_macos_process metrics.json -- executable [arguments...]
// Read final process counters before reaping the child, including all threads.
// Retired instructions measure work without assuming a constant CPU frequency.
#include <errno.h>
#include <inttypes.h>
#include <libproc.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

static volatile sig_atomic_t child_pid = -1;

static void forward_signal(int signal_number)
{
    if (child_pid > 0)
        kill(child_pid, signal_number);
}

int main(int argc, char **argv)
{
    if (argc < 4 || strcmp(argv[2], "--") != 0) {
        fprintf(stderr, "Usage: %s metrics.json -- executable [arguments...]\n", argv[0]);
        return 125;
    }
    const pid_t pid = fork();
    if (pid < 0) { perror("fork"); return 125; }
    if (pid == 0) {
        execvp(argv[3], argv + 3);
        perror("execvp");
        _exit(127);
    }
    child_pid = pid;
    signal(SIGINT, forward_signal);
    signal(SIGTERM, forward_signal);
    siginfo_t status = {0};
    int waited;
    do { waited = waitid(P_PID, pid, &status, WEXITED | WNOWAIT); } while (waited < 0 && errno == EINTR);
    if (waited < 0) { perror("waitid"); kill(pid, SIGKILL); waitpid(pid, NULL, 0); return 125; }

    struct rusage_info_v4 usage = {0};
    const int measured = proc_pid_rusage(pid, RUSAGE_INFO_V4, (rusage_info_t *)&usage);
    const int measurement_errno = errno;
    int exit_status;
    while (waitpid(pid, &exit_status, 0) < 0) {
        if (errno != EINTR) { perror("waitpid"); return 125; }
    }
    child_pid = -1;
    if (measured != 0 || usage.ri_instructions == 0) {
        fprintf(stderr, "Retired instruction counters unavailable: %s\n",
                measured ? strerror(measurement_errno) : "zero instruction count");
        return 125;
    }
    FILE *output = fopen(argv[1], "w");
    if (!output) { perror("metrics output"); return 125; }
    const int written = fprintf(output,
        "{\"retired_instructions\":%" PRIu64 ",\"cycles\":%" PRIu64
        ",\"peak_physical_bytes\":%" PRIu64 ",\"disk_read_bytes\":%" PRIu64
        ",\"disk_written_bytes\":%" PRIu64 "}\n",
        usage.ri_instructions, usage.ri_cycles, usage.ri_lifetime_max_phys_footprint,
        usage.ri_diskio_bytesread, usage.ri_diskio_byteswritten);
    const int closed = fclose(output);
    if (written < 0 || closed != 0) { perror("writing metrics"); return 125; }
    return WIFEXITED(exit_status) ? WEXITSTATUS(exit_status) : 128 + WTERMSIG(exit_status);
}
