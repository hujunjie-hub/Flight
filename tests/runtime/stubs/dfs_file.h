#ifndef TEST_DFS_FILE_H
#define TEST_DFS_FILE_H

#include <fcntl.h>
#include <stdio.h>
#include <sys/types.h>

#ifndef O_DIRECTORY
#define O_DIRECTORY 0x02000000
#endif
#define rt_snprintf snprintf

struct dfs_file {
    unsigned stream;
};

void fd_init(struct dfs_file *file);
int dfs_mount(const char *device, const char *path, const char *fs,
              unsigned long flags, const void *data);
int dfs_unmount(const char *path);
int dfs_file_open(struct dfs_file *file, const char *path, int flags);
int dfs_file_close(struct dfs_file *file);
ssize_t dfs_file_write(struct dfs_file *file, const void *data, size_t length);
int dfs_file_flush(struct dfs_file *file);

#endif
