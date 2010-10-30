/**
 * 2010 instructions (see 2010 wiki for CTC setup first):
 * ROBOT=sniper # change this to your preferred Nao
 * $CTC_DIR/cross/geode/bin/i586-linux-gcc -fPIC ioctltrap.c -g -c -Wall --sysroot=$CTC_DIR/staging/geode-linux/
 * $CTC_DIR/cross/geode/bin/i586-linux-gcc -shared -Wl,-soname,libioctltrap.so.1 -o libioctltrap.so.1.0 -lc -ldl ioctltrap.o --sysroot=$CTC_DIR/staging/geode-linux/
 * rsync -aP libioctltrap.so.1.0 naoqi.patch nao@$ROBOT.local:
 * ssh nao@$ROBOT.local
 * grep ioctl /usr/bin/naoqi>/dev/null || su -c 'patch -p0' < naoqi.patch
 * naoqi --ioctltrap
 * # then monitor /tmp/ioctltrap.txt
 **/

#include <stdio.h>
#define _GNU_SOURCE
#include <dlfcn.h>
#undef _GNU_SOURCE
#include <stdarg.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <bn/i2c/i2c-dev.h>
#include <sys/stat.h>
#include <linux/videodev2.h>

/**
 * file descriptor for debug output
 */
static FILE *fd;


/**
 * indicates that this function is run when the library is loaded
 */
static void _libioctltrap_init(void) __attribute__((constructor));

/**
 * initializes the library
 *
 * prints out a debug message
 */
static void _libioctltrap_init(void) {
  printf("libioctltrap: starting up\n");
}

/**
 * indicates that this function is run when the library is unloaded
 */
static void _libioctltrap_fini(void) __attribute__((destructor));

/**
 * uninitializes the library
 *
 * prints out a debug message
 */
static void _libioctltrap_fini(void) {
  printf("libioctltrap: shutting down\n");
}

/**
 * returns the minimum of two numbers/functions
 */
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))

/**
 * prints out the bytes that will be written to/were read from the device
 *
 * @param request the encoded ioctl request (dir, type, nr, size)
 * @param arg     the bytes
 */
void printbytes(const int request, const unsigned char *arg){
  if(_IOC_TYPE(request) == 'V'){
    int byte;
    for(byte = 0; byte < _IOC_SIZE(request); ++byte)
      fprintf(fd, "%02x", arg[byte]);
  }
}

/**
 * prints out the bytes that are associated with the i2c call, if the associated
 * direction matches
 *
 * @param arg     the bytes
 * @param dir the direction
 */
void printi2c_smbus(const unsigned char *arg, char dir){
   const struct i2c_smbus_ioctl_data *args =
         (const struct i2c_smbus_ioctl_data *)arg;
   if(dir != (args->read_write?'R':'W'))
      return;
   fprintf(fd, "%c command:%d size:%d ", args->read_write?'R':'W',
           args->command, args->size);
   if(args->data) //data can be NULL with write byte
      switch(args->size){
         case I2C_SMBUS_QUICK:
            break;
         case I2C_SMBUS_BYTE:
         case I2C_SMBUS_BYTE_DATA:
            fprintf(fd, "%02x", args->data->byte);
            break;
         case I2C_SMBUS_WORD_DATA:
         case I2C_SMBUS_PROC_CALL:
            fprintf(fd, "%04x", args->data->word);
            break;
         case I2C_SMBUS_BLOCK_DATA:
         case I2C_SMBUS_I2C_BLOCK_BROKEN:
         case I2C_SMBUS_BLOCK_PROC_CALL:
         case I2C_SMBUS_I2C_BLOCK_DATA:
         {
            fprintf(fd, "length: %d ", args->data->block[0]);
            int i2c_smbus_block_index;
            for(i2c_smbus_block_index = 1;
                i2c_smbus_block_index < 2 + MIN(args->data->block[0],
                      I2C_SMBUS_BLOCK_MAX);
                ++i2c_smbus_block_index)
               fprintf(fd, "%02x", args->data->block[i2c_smbus_block_index]);
         }
         break;
         default:
            fprintf(fd, "odd size");
            break;
      }
}

/**
 * prints all kind of useful stats for a file descriptor
 *
 * @param d the file descriptor
 */
void printfstat(int d) {
   struct stat sb;

   if (fstat(d, &sb) == -1) {
      perror("stat");
   }

   fprintf(fd, "File type:                ");

   switch (sb.st_mode & S_IFMT) {
      case S_IFBLK:  fprintf(fd, "block device\n");            break;
      case S_IFCHR:  fprintf(fd, "character device\n");        break;
      case S_IFDIR:  fprintf(fd, "directory\n");               break;
      case S_IFIFO:  fprintf(fd, "FIFO/pipe\n");               break;
      case S_IFLNK:  fprintf(fd, "symlink\n");                 break;
      case S_IFREG:  fprintf(fd, "regular file\n");            break;
      case S_IFSOCK: fprintf(fd, "socket\n");                  break;
      default:       fprintf(fd, "unknown?\n");                break;
   }

   fprintf(fd, "I-node number:            %ld\n", (long) sb.st_ino);

   fprintf(fd, "Mode:                     %lo (octal)\n",
          (unsigned long) sb.st_mode);

   fprintf(fd, "Link count:               %ld\n", (long) sb.st_nlink);
   fprintf(fd, "Ownership:                UID=%ld   GID=%ld\n",
          (long) sb.st_uid, (long) sb.st_gid);

   fprintf(fd, "Preferred I/O block size: %ld bytes\n",
          (long) sb.st_blksize);
   fprintf(fd, "File size:                %lld bytes\n",
          (long long) sb.st_size);
   fprintf(fd, "Blocks allocated:         %lld\n",
          (long long) sb.st_blocks);

   fprintf(fd, "Last status change:       %s", ctime(&sb.st_ctime));
   fprintf(fd, "Last file access:         %s", ctime(&sb.st_atime));
   fprintf(fd, "Last file modification:   %s", ctime(&sb.st_mtime));
}

/**
 * returns the inode for a file descriptor
 *
 * @param d the file descriptor
 * @return the inode
 */
long inode(int d) {
   struct stat sb;

   if (fstat(d, &sb) == -1) {
      perror("stat");
   }

   return (long) sb.st_ino;
}

/**
 * The  ioctl()  function  manipulates the underlying device parameters of
 * special files.  In particular, many operating characteristics of  character
 * special  files  (e.g., terminals) may be controlled with ioctl() requests.
 * The argument d must be an open file descriptor.
 */
int ioctl(int d, unsigned long int request, ...)
{
  static int (*orig_ioctl)(int, int, ...) = NULL;
  //static pthread_mutex_t mutexwrite;
  if(!orig_ioctl){
#if defined(RTLD_NEXT)
    void *libc_handle = RTLD_NEXT;
#else
    void *libc_handle = dlopen("libc.so.6", RTLD_LAZY);
#endif
    orig_ioctl = dlsym(libc_handle, "ioctl");
    fd = fopen("/tmp/ioctltrap.txt", "a");
    //pthread_mutex_init(&mutexwrite, NULL);
  }
  va_list opt;
  unsigned char *arg;
  va_start(opt, request);
  arg = va_arg(opt, unsigned char *);
  va_end(opt);
  //pthread_mutex_lock (&mutexwrite);
  fprintf(fd, "ioctl %d\tinode: %ld\tdir: %lu\ttype:%lu\tnr: %lu\tsize: %lu ", d,
          inode(d), _IOC_DIR(request),_IOC_TYPE(request), _IOC_NR(request),
          _IOC_SIZE(request));
  //if it's going to write, spit out the bytes
  if((_IOC_DIR(request) & _IOC_WRITE) && request != VIDIOC_G_CTRL){
    printbytes(request, arg);
  }
  if(request == I2C_SMBUS)
     printi2c_smbus(arg, 'W');
  if(request == I2C_SLAVE)
     fprintf(fd, "%p", arg);
  //pthread_mutex_unlock (&mutexwrite);
  int retval = orig_ioctl(d, request, arg);
  //if it's going to read, and didn't write, spit out the bytes
  if(_IOC_DIR(request) == _IOC_READ || request == VIDIOC_G_CTRL){
    printbytes(request, arg);
  }
  if(request == I2C_SMBUS)
     printi2c_smbus(arg, 'R');
  fprintf(fd, "\n");
  fflush(fd);
  return retval;
}

/**
 * Apply or remove an advisory lock on the open file specified by d.  The
 * argument operation is one of the following:
 *     LOCK_SH  Place a shared lock.  More than one  process  may  hold  a
 *              shared lock for a given file at a given time.
 *     LOCK_EX  Place  an  exclusive  lock.   Only one process may hold an
 *              exclusive lock for a given file at a given time.
 *     LOCK_UN  Remove an existing lock held by this process.
 */
int flock(int d, int operation){
   static int (*orig_flock)(int, int) = NULL;
   if(!orig_flock){
#if defined(RTLD_NEXT)
      void *libc_handle = RTLD_NEXT;
#else
      void *libc_handle = dlopen("libc.so.6", RTLD_LAZY);
#endif
      orig_flock = dlsym(libc_handle, "flock");
      fd = fopen("/tmp/ioctltrap.txt", "a");
   }
   fprintf(fd, "flock %d\tinode: %ld\top: %d", d, inode(d), operation);
   int retval = orig_flock(d, operation);
   fprintf(fd, "\n");
   fflush(fd);
   return retval;
}

/**
 * fcntl() performs one of the operations described in the manpage on the open
 * file descriptor fd.  The operation is determined by cmd.
 */
int fcntl (int __fd, int __cmd, ...){
   static int (*orig_fcntl)(int, int, ...) = NULL;
   if(!orig_fcntl){
#if defined(RTLD_NEXT)
      void *libc_handle = RTLD_NEXT;
#else
      void *libc_handle = dlopen("libc.so.6", RTLD_LAZY);
#endif
      orig_fcntl = dlsym(libc_handle, "fcntl");
      fd = fopen("/tmp/ioctltrap.txt", "a");
   }
   va_list opt;
   unsigned char *arg;
   va_start(opt, __cmd);
   arg = va_arg(opt, unsigned char *);
   va_end(opt);
   fprintf(fd, "fcntl %d\tinode: %ld\tcmd: %d", __fd, inode(__fd), __cmd);
   int retval = orig_fcntl(__fd, __cmd, arg);
   fprintf(fd, "\n");
   fflush(fd);
   return retval;
}

/**
 * Given a pathname for a file, open() returns a file descriptor, a small,
 * non-negative integer for  use  in  subsequent  system  calls  (read(2),
 * write(2), lseek(2), fcntl(2), etc.).  The file descriptor returned by a
 * successful call will be the lowest-numbered file  descriptor  not  cur-
 * rently open for the process.
 */
int open(const char *pathname, int flags, ...){
   static int (*orig_open)(int, int, ...) = NULL;
   if(!orig_open){
#if defined(RTLD_NEXT)
      void *libc_handle = RTLD_NEXT;
#else
      void *libc_handle = dlopen("libc.so.6", RTLD_LAZY);
#endif
      orig_open = dlsym(libc_handle, "open");
      fd = fopen("/tmp/ioctltrap.txt", "a");
   }
   va_list opt;
   unsigned char *arg;
   va_start(opt,flags);
   arg = va_arg(opt, unsigned char *);
   va_end(opt);
   fprintf(fd, "open %s\tflags: 0%o", pathname, flags);
   int retval = orig_open(pathname, flags, arg);
   fprintf(fd, "\n");
   fflush(fd);
   return retval;
}
