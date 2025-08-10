Build

#### qt-6.5.3 source build

```
make sure ninja and python3.9 are under PATH

set PATH=D:\Python39;%PATH%

where python
D:\Python39\python.exe
C:\Users\nmam\AppData\Local\Microsoft\WindowsApps\python.exe

where ninja
C:\Windows\System32\ninja.exe
C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe

\qt-everywhere-src-6.5.3\configure.bat -prefix D:\qt-6.5.3\install -skip qtconnectivity -nomake examples -nomake tests -skip speech -skip scxml -skip qtsensors -skip qtserialbus -skip qtserialport -skip qtspeech -skip qtdoc -skip qtandroidextras -skip qtcanvas3d  -skip qtdatavis3d -skip qttranslations -skip qtvirtualkeyboard -skip qtwayland -skip qtwebchannel -skip qtwebengine -skip qt3d -skip qtcharts -skip qtactiveqt -skip qtlocation -skip qtmultimedia -skip qtquick3d -skip qtgraphs -skip qtwebview -skip qtcoap -skip qtquickeffectmaker -skip qtscxml -skip qtquick3dphysics -skip qtmqtt -skip qtpositioning -skip qtlanguageserver -skip qtlottie -skip qtgrpc -skip qtopcua -release

cmake --build . --parallel
cmake --install .
Qt will be installed into 'D:/qt-6.5.3/install'
```

#### Build

```sh
cd auv/app && mkdir build && cd build
SET Qt6_DIR=D:\qt-6.5.3\install\lib\cmake\Qt6
SET Qt6QmlTools_DIR=D:\qt-6.5.3\install\lib\cmake\Qt6QmlTools
cmake -DNANOPB_SRC_ROOT_FOLDER=D:/nanopb-0.4.9.1 ..
cmake --build . --config Release

Run
SET PATH=%PATH%;D:\qt-6.5.3\install\bin
build\Release\app.exe
```

#### Deploy:

```sh
windeployqt --qmldir E:\mcu\app\qml E:\mcu\app\build\Release\app.exe
```