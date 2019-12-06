# IAR_TME_Roborobo

## Installation
Download and extract (without deleting already existing files)
```
git clone https://github.com/ThibautSF/IAR_TME_Roborobo.git
tar xvkfz roborobo3_iar_2019rc1.tgz
```

Check if dependencies are installed (see `roborobo3_iar_2019/_INSTALL.txt`)

Make
```
cd roborobo3_iar_2019/
make -j8
```

Test
```
./roborobo -l config/tutorial.properties
```

