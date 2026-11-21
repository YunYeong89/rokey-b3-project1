/* Keep DRFL's TCP authentication consistent with the endpoint seen by the robot.
 * Only the configured robot's TCP 12345 connection is relayed; RT UDP is direct.
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

static atomic_int robot_fd = -1;
static struct sockaddr_in public_address;

int connect(int fd, const struct sockaddr *address, socklen_t size) {
    int (*original)(int, const struct sockaddr *, socklen_t) = dlsym(RTLD_NEXT, "connect");
    const char *robot_ip = getenv("COBOT_ROBOT_IP");
    const char *relay_port = getenv("COBOT_ROBOT_RELAY_PORT");
    const struct sockaddr_in *target = (const struct sockaddr_in *)address;
    if (!robot_ip || !relay_port || address->sa_family != AF_INET ||
        target->sin_addr.s_addr != inet_addr(robot_ip) || target->sin_port != htons(12345))
        return original(fd, address, size);

    struct addrinfo hints = {.ai_family = AF_INET, .ai_socktype = SOCK_STREAM}, *relay;
    if (getaddrinfo("host.docker.internal", relay_port, &hints, &relay)) {
        errno = EHOSTUNREACH;
        return -1;
    }
    int flags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
    int result = original(fd, relay->ai_addr, relay->ai_addrlen);
    freeaddrinfo(relay);
    unsigned char endpoint[6];
    struct pollfd waiter = {.fd = fd, .events = POLLIN};
    if (!result && poll(&waiter, 1, 5000) > 0 &&
        recv(fd, endpoint, sizeof(endpoint), MSG_WAITALL) == sizeof(endpoint)) {
        public_address.sin_family = AF_INET;
        memcpy(&public_address.sin_addr, endpoint, 4);
        memcpy(&public_address.sin_port, endpoint + 4, 2);
        robot_fd = fd;
    } else {
        result = -1;
        errno = ECONNABORTED;
    }
    int error = errno;
    fcntl(fd, F_SETFL, flags);
    errno = error;
    return result;
}

int getsockname(int fd, struct sockaddr *address, socklen_t *size) {
    int (*original)(int, struct sockaddr *, socklen_t *) = dlsym(RTLD_NEXT, "getsockname");
    if (fd != robot_fd)
        return original(fd, address, size);
    memcpy(address, &public_address,
           *size < sizeof(public_address) ? *size : sizeof(public_address));
    *size = sizeof(public_address);
    return 0;
}

int close(int fd) {
    int (*original)(int) = dlsym(RTLD_NEXT, "close");
    if (fd == robot_fd)
        robot_fd = -1;
    return original(fd);
}
