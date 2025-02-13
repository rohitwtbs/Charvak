cdef extern from "openxr/openxr.h":
    ctypedef unsigned long long XrInstance
    ctypedef unsigned long long XrSystemId
    ctypedef unsigned long long XrSession
    ctypedef unsigned long long XrSpace
    ctypedef unsigned int XrResult

    struct XrApplicationInfo:
        char applicationName[128]
        int applicationVersion
        char engineName[128]
        int engineVersion
        int apiVersion

    struct XrInstanceCreateInfo:
        int type
        XrApplicationInfo applicationInfo

    XrResult xrCreateInstance(XrInstanceCreateInfo* createInfo, XrInstance* instance)
    XrResult xrDestroyInstance(XrInstance instance)
