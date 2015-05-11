# 6.834

## Compiling 
You'll need Bullet physics engine and Eigen math library. If using Linux, install libeigen3-dev and libbullet-dev. Locations of the library files are hardcoded in the CMakeLists.txt files. 
```
cd bin/
cmake .. -DCMAKE_BUILD_TYPE=RELEASE
```

## Tests
There are a few tests. Executables should be generated under bin/tests/test_x. Note that I've hardcoded locations of mesh files, so you'll probably get errors unless you change them. Will try to fix this tomorrow. 
