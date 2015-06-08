# imx6tinyrex_uboot_v2014_10
mx6q/dl/s tinyrex u-boot v2014.10 

# Download repository
    git clone https://github.com/voipac/imx6tinyrex_uboot_v2014_10
    cd imx6tinyrex_uboot_v2014_10

# Setup cross compiler
    export PATH="/opt/gcc-linaro-arm-linux-gnueabihf-4.8-2014.04_linux/bin:~/workdir/bin:$PATH"
    export CROSS_COMPILE=arm-linux-gnueabihf-
    export ARCH=arm

# Build (imx6s tinyrex)
    make distclean
    make mx6stinyrex_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6s-tinyrex.imx
    
# Build (imx6dl tinyrex)
    make distclean
    make mx6dltinyrex_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6dl-tinyrex.imx

# Build (imx6q tinyrex)
    make distclean
    make mx6qtinyrex_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6q-tinyrex.imx

# Build (imx6q rex)
    make distclean
    make mx6qrex_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6q-rex.imx
