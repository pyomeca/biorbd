from distutils.core import setup, Extension

module1 = Extension("pyorbdl",
                    include_dirs=['S2MSource/include'],
                    libraries=['S2M_Contacts',
                               'S2M_GeneralizedCoordinates',
                               'S2M_Markers',
                               'S2M_MusclePaths',
                               'S2M_Muscles',
                               'S2M_MusculoSkeletalModel',
                               'S2M_RigidBody'],
                    library_dirs=['pyomeca/thirdparty/S2MLib'],
                    runtime_library_dirs=['pyomeca/thirdparty/S2MLib', '../pyomeca/thirdparty/S2MLib'],
                    sources=['pyorbdl.cpp']
                    , define_macros=[('CXXFLAGS', '-O0 -g')]
                    )

setup(name="pyorbdl",
      version="1.0",
      description="Interface to RBDL",
      author="Pariterre",
      ext_modules=[module1]
      )

