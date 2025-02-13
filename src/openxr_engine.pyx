from libc.string cimport strcpy
from openxr_bindings cimport XrInstance, XrInstanceCreateInfo, XrApplicationInfo, xrCreateInstance, xrDestroyInstance

cdef class OpenXREngine:
    cdef XrInstance xr_instance

    def __cinit__(self):
        self.xr_instance = 0

    def initialize(self):
        cdef XrApplicationInfo app_info
        strcpy(app_info.applicationName, b"VRGameEngine")
        app_info.applicationVersion = 1
        app_info.engineVersion = 1
        app_info.apiVersion = 1

        cdef XrInstanceCreateInfo create_info
        create_info.type = 1  # XR_TYPE_INSTANCE_CREATE_INFO
        create_info.applicationInfo = app_info

        if xrCreateInstance(&create_info, &self.xr_instance) != 0:
            raise RuntimeError("Failed to create OpenXR instance!")

    def cleanup(self):
        if self.xr_instance:
            xrDestroyInstance(self.xr_instance)
            self.xr_instance = 0
