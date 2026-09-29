// RAII wrapper for Linux file descriptors
// Provides automatic resource management and exception safety

#ifndef HAL_FILE_DESCRIPTOR_H_
#define HAL_FILE_DESCRIPTOR_H_

#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <stdexcept>

class FileDescriptor {
 private:
  int fd;

 public:
  // Constructor - opens file and acquires resource
  FileDescriptor(const std::string& path, int flags) 
      : fd(open(path.c_str(), flags)) {
    if (fd < 0) {
      throw std::runtime_error("Failed to open file: " + path);
    }
  }

  // Destructor - automatically releases resource
  ~FileDescriptor() {
    if (fd >= 0) {
      close(fd);
    }
  }

  // Move constructor - transfer ownership
  FileDescriptor(FileDescriptor&& other) noexcept 
      : fd(other.fd) {
    other.fd = -1;
  }

  // Move assignment - transfer ownership
  FileDescriptor& operator=(FileDescriptor&& other) noexcept {
    if (this != &other) {
      if (fd >= 0) {
        close(fd);
      }
      fd = other.fd;
      other.fd = -1;
    }
    return *this;
  }

  // Delete copy operations - file descriptors should not be copied
  FileDescriptor(const FileDescriptor&) = delete;
  FileDescriptor& operator=(const FileDescriptor&) = delete;

  // Get the raw file descriptor
  int get() const { return fd; }

  // Check if file descriptor is valid
  bool is_valid() const { return fd >= 0; }
};

#endif /* HAL_FILE_DESCRIPTOR_H_ */
