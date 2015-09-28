## imx6tinyrex_uboot_v2014_10
mx6q/dl/s tinyrex u-boot v2014.10

mx6q/dl/d rex u-boot v2014.10 

## Download repository
    git clone https://github.com/voipac/imx6tinyrex_uboot_v2014_10
    cd imx6tinyrex_uboot_v2014_10

## Setup cross compiler
    export PATH="/opt/gcc-linaro-arm-linux-gnueabihf-4.8-2014.04_linux/bin:~/workdir/bin:$PATH"
    export CROSS_COMPILE=arm-linux-gnueabihf-
    export ARCH=arm

## Build
#### Build (imx6s tinyrex)
    make distclean
    make mx6stinyrex_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6s-tinyrex.imx
    
#### Build (imx6dl tinyrex)
    make distclean
    make mx6dltinyrex_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6dl-tinyrex.imx

#### Build (imx6q tinyrex)
    make distclean
    make mx6qtinyrex_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6q-tinyrex.imx

#### Build (imx6dl rex)
    make distclean
    make mx6dlrex_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6dl-rex.imx

#### Build (imx6d rex)
    make distclean
    make mx6drex_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6d-rex.imx

#### Build (imx6q rex)
    make distclean
    make mx6qrex_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6q-rex.imx

## IMPORTANT
    u-boot-imx6q-rex.imx must be flashed into spi flash at offset 0x400.
    
## Some lines that might be helpful
    setenv ipaddr 192.168.0.150
    setenv serverip 192.168.0.1
    tftp 0x17800000 imx6/u-boot-imx6q-rex.bin
    go 0x17800000

    mw.b 0x10800000 0xFF 0x80000;
    tftp 0x10800000 imx6/u-boot-imx6q-rex.imx;
    sf probe 2:2;sf erase 0x0 0x80000;sf write 0x10800000 0x400 0x7fc00

  
