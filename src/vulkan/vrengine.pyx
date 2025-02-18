# distutils: language = c++
from libc.stdint cimport uint32_t
from libc.string cimport memset
cimport cython

# Vulkan Imports
cdef extern from "vulkan/vulkan.h":
    ctypedef uint32_t VkResult
    ctypedef uint32_t VkInstance
    VkResult vkCreateInstance(...)  # Vulkan instance creation placeholder
    VkResult vkEnumeratePhysicalDevices(...)

# Initialize Vulkan
def init_vulkan():
    print("Initializing Vulkan...")

    cdef VkInstance instance
    res = vkCreateInstance(&instance)
    if res != 0:
        raise RuntimeError("Failed to initialize Vulkan!")

    print("Vulkan Initialized!")

def run():
    init_vulkan()
