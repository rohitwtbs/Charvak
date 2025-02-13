from setuptools import setup
from Cython.Build import cythonize
import os

openxr_path = "/opt/homebrew"  # Adjust if needed

setup(
    ext_modules=cythonize(
        "src/openxr_engine.pyx",
        compiler_directives={"language_level": "3"}
    ),
    include_dirs=[f"{openxr_path}/include"],
    library_dirs=[f"{openxr_path}/lib"],
    libraries=["openxr_loader"],  # Link with OpenXR
)
