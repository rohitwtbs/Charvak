# example.pyx
cimport cython
from libc.stdio cimport printf
from libc.time cimport clock, CLOCKS_PER_SEC
from libc.time cimport clock_t #add this line.

cdef void cpu_intensive_task(int iterations) nogil:
    cdef int i
    cdef double result = 0.0
    for i in range(iterations):
        result += i * i

cdef void io_bound_task(int seconds):
    cdef clock_t start_time = clock()
    cdef clock_t end_time = seconds * CLOCKS_PER_SEC + start_time
    while clock() < end_time:
        pass #simulate i/o

def run_tasks(int cpu_iterations, int io_seconds):
    # CPU-bound task (nogil)
    printf(b"Starting CPU task...\n")
    cpu_intensive_task(cpu_iterations)
    printf(b"CPU task finished.\n")

    # I/O-bound task (GIL acquired by default)
    printf(b"Starting I/O task...\n")
    io_bound_task(io_seconds)
    printf(b"I/O task finished.\n")

    # Example of acquiring and releasing the GIL manually (less common, but possible)
    printf(b"Starting manual GIL manipulation...\n")
    with cython.nogil:
        printf(b"Inside nogil block...\n")
        with cython.gil: #now acquire gil to print from python
            print("Python print inside gil block")
        printf(b"Back inside nogil block.\n")
    printf(b"Manual GIL manipulation finished.\n")