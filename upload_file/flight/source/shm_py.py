import mmap
from ctypes import *
import sys
import time

libc = CDLL("libc.so.6", use_errno = True)
shmget = libc.shmget
shmget.argtypes = [c_int, c_size_t, c_int]
shmget.restype = c_int
shmat = libc.shmat
shmat.argtypes = [c_int, POINTER(c_void_p), c_int]
shmat.restype = c_void_p

a = 1.1
b = 2.2
c = 3.3
a = str(a)
b = str(b)
c = str(c)
shared_info = a+'!'+b+'!'+c+'!'
shared_info = shared_info.encode()
print(shared_info)
print(sys.getsizeof(shared_info))

SHM_ADDR = 0x100
shm_id = shmget(c_int(SHM_ADDR), sys.getsizeof(shared_info), c_int(0o1000|0o666))
if shm_id == -1:
    print("py shmget failed")
    sys.exit()
    
shm_addr = shmat(shm_id, None, 0)
if shm_addr == -1:
    print("py shmat failed")
    sys.exit()

libc.strcpy(cast(shm_addr, c_char_p), create_string_buffer(shared_info))

print("py done")
sys.exit()


