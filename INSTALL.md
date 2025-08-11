# Comedi Installation

## Introduction

See the *[README.md][]* file for general information about
**[Comedi][]** and where to obtain the source code.

Each instance of the **Comedi** installation is configured to be build
for a specific installation of the **[Linux][]** kernel, so when a new
kernel is installed, the **Comedi** installation needs to be
reconfigured and rebuilt for that kernel.  Most distributions support
building external modules for the kernels they install.

There have not been any official releases since **Comedi 0.7.76** on
2008-01-28, which will not build for any recent **Linux** kernels.
However, it is possible to generate unofficial releases from the **Git**
sources; see the section *[Generating Unofficial Releases][genrel]* for
details.

**Comedi** can be built using either the sources from the **Git**
repository after some preparation, or the sources from an unofficial
release.

It is possible to configure the operating system to build and install
the **Comedi** modules automatically when the kernel is updated, by
using an unofficial release to create **[Akmods][]** packages for Red
Hat based distributions, or to use **[Dynamic Kernel Module System
(DKMS)][dkms]** for other distributions.  These options are described in
the section *[Automatic Installation][autoins]*.

### Build Failures

**Comedi** may fail to build for some kernels, particularly for brand
new kernel releases, or due to certain vendors (mostly Red Hat)
backporting kernel API features from new kernel release series to older
series.  For brand new kernels, pull updates from the **Git** repository
and try building from the updated sources, because the problem may have
already been fixed in the latest sources.  Build failures for older
kernels (including vendor's long term kernels) are less likely to have
been fixed in the updated sources.

Please do try to build from the latest sources before reporting problems
on the [Comedi mailing list][mail]. When reporting problems capture
output from the build in a text file and include that in the email, and
include other details such as the **Linux** distribution and **Linux**
kernel version.  Build output from the shell session can be captured by
redirecting the standard output and standard error output to a text
file:

```
some command > errors.txt 2>&1
```

## Obtaining Sources From Git

> [!NOTE]
> If the **git** command cannot be used to clone the **Comedi** **Git**
> repository remotely, it is possible to download a snapshot.  See the
> section *[Downloading A Snapshot Of The Sources][snap]*.

It is recommended to use the **git clone** command from a shell session
to clone the **Git** repository for the **Comedi** sources:

```
git clone https://github.com/Linux-Comedi/comedi.git
```

The above command will create the sub-directory *comedi* in the current
working directory and check out the "master" branch of the **Comedi**
**Git** repository into that sub-directory.

Change directory to the *comedi* sub-directory for further operations on
the **Git** repository sources:

```
cd comedi
```

> [!NOTE]
> The checked out sources in this sub-directory will need to be prepared
> before Comedi can be configured to be built.  See the section
> *[Preparing The Sources From Git][prep]*.

When using the "master" branch, the sources can be brought up to date
using the **git pull** command:

```
git pull
```

See the **[Git web site][git]** for more information about **Git**.

### Downloading A Snapshot Of The Sources

If the **git** command is unavailable, or the system has no network
access, a tarball file containing a snapshot of the latest sources can
be downloaded (using a system with network access) by visiting the link
<https://github.com/Linux-Comedi/comedi/tarball/master>.  The downloaded
file will have a weird name something like
*Linux-Comedi-comedi-v0.7.76.1-373-g58cbbf6.tar.gz*, depending on the
state of the **Comedi** **Git** repository at the time.  This can be
unpacked using the **tar** command, and will create a sub-directory with
a similar name (but without the *.tar.gz* extension) containing the
snapshot of the sources.  For example:

```
tar zxvf Linux-Comedi-comedi-v0.7.76.1-373-g58cbbf6.tar.gz
```

Change directory to that sub-directory for further operations on the
sources.  For example:

```
cd Linux-Comedi-comedi-v0.7.76.1-373-g58cbbf6
```

> [!NOTE]
> The checked out sources in this sub-directory will need to be prepared
> before **Comedi** can be configured to be built.  See the section
> *[Preparing The Sources From Git][prep]*.

To check for updated sources, visit
<https://github.com/Linux-Comedi/comedi/activity?ref=master>. To update
the sources, download and unpack another snapshot.

## Preparing The Sources From Git

> [!NOTE]
> This is not needed when using the sources from an unofficial (or
> official) release tarball, because the sources therein have already
> been prepared for use.

The **Comedi** project uses **[Autoconf][]** and **[Automake][]** to
generate the configuration scripts and Makefiles used to configure and
build the project.  The generated files are not stored in the **Git**
repository, so the **Comedi** sources from the **Git** repository need
to be prepared before the **Comedi** build can be configured.

The **Linux** distribution should have packages for **autoconf** and
**automake** that can be installed using the distribution's package
manager.

The minimum version requirements are:

* **automake** >= 1.6
* **autoconf** >= 2.64

There is a shell script in the top-level **Comedi** source directory
called **autogen.sh** that should have been marked as executable when
the Git repository was cloned or the snapshot was unpacked.

To prepare the **Comedi** sources using a shell session, change
directory to the top-level **Comedi** source directory, and run the
following command:

```
./autogen.sh
```

If there is a "Permission denied" error (**EPERM**) when running the
script, check that the script is executable and that the top-level
source directory and its sub-directories are readable and writeable.

If there is an "autoreconf: command not found" error from running the
script, check that the **autoconf** package is installed.

If there a an error from **autoreconf** about failing to run
**aclocal**, check that the **automake** package is installed.

If there are no errors running the **autogen.sh** script, then the sources
have been prepared successfully.

## In-Tree And Out-Of-Tree Builds

It is possible to configure and build **Comedi** in the top-level source
directory itself (an *in-tree* build), or in a separate *build*
directory (an *out-of-tree* build).  An out-of-tree build has the
advantage of allowing the same source directory to be shared by several
builds.  **Comedi** will be configured and built in the working
directory from which the **configure** script is run. (See section
*[Configuration][config]*.)  A relative or absolute pathname may be used
to specify the location of the **configure** script.  For an in-tree
build, this will be **`./configure`**.

## Configuration

Configuration of the **Comedi** build is performed by running the
**configure** script contained in the top-level **Comedi** source
directory.  It is run by a relative or absolute pathname from the
*build* directory, which will be the same as top-level **Comedi** source
directory for an in-tree build, but will be a different directory for an
out-of-tree build. (See section *[In-Tree And Out-Of-Tree
Builds][builddir]*.)

> [!NOTE]
> If the **configure** script does not exist in the top-level **Comedi**
> source directory, see section *[Preparing The Sources From
> Git][prep]*.

The **configure** script supports various command-line options to
configure the build.  Run it with the **`--help`** option to list the
available configuration options.  The **`--with-linuxdir`** option is
particularly useful, as it allows you to specify the location of your
**Linux** kernel build directory.  If you are building for a Linux kernel
patched for **[RTAI][]** or **RTLinux**, the **`--with-rtaidir`** or
**`--with-rtlinuxdir`** options allow you to specify the location of
your **RTAI** or **RTLinux** source directory.

> [!NOTE]
> **RTLinux** is a dead project, not to be confused with **Real-Time
> Linux** (**PREEMPT_RT**).

A successful run of the **configure** script requires at least the
following:

* the **awk** program or equivalent (usually **gawk**);
* a C compiler (usually **gcc**, but can be overridden with the *`CC`*
  environment variable);
  compiler that was used to build the **Linux** kernel);
* the **make** program;
* the **strip** program (usually installed by the **binutils** package);
* the **depmod** program (usually installed by the **kmod** or
  **modutils** package);
* **Linux** kernel headers for the desired kernel (usually installed by
  a **kernel-devel** or **linux-headers** package specific to the kernel
  version);

> [!NOTE]
> The C compiler needs to be compatible with the C compiler that the
> kernel was built with.

## Building

Once the **Comedi** build has been configured, run **`make`** in the build
directory to build the modules.  If the system has multiple CPUs, use
the **`-j $(nproc)`** option to increase the number of parallel jobs
according to the number of CPUs.

## Installation

After a successful build, install using **`make install`** as user
*root*, or via the **`sudo`** command. For example:

```
sudo make install
```

> [!NOTE]
> If the kernel is digitally signed, there may be errors signing the
> modules, but the unsigned modules should still be installed, and
> should be loadable as long as the system is not enforcing the use of
> signed modules.  It is possible to enroll and use your own signing
> keys, but that is beyond the scope of this article.

## Post-Installation Steps

### Module Parameters

If **Comedi** will be used with "legacy" devices that do not support
auto-configuration, such as drivers for ISA cards, or dummy test devices
for the **comedi_test** kernel module, then the **comedi** kernel module
needs to be loaded with the **comedi_num_legacy_minors** module
parameter set to the number of **Comedi** device minor numbers to be
reserved for legacy devices.  If special device files in */dev* are
managed dynamically by **udev**, the special device files for the
reserved legacy devices will be created when the **comedi** module is
loaded.  For example if the **comedi** module is loaded with the
parameter **`comedi_num_legacy_minors=4`**, the special device files
*/dev/comedi0*, */dev/comedi1*, */dev/comedi2*, and */dev/comedi3* will
be created when the module is loaded.  Some **Comedi** low-level drivers
(for PCI and USB devices) will auto-configure a new **Comedi** device
when matching hardware is detected.  These will be created with minor
device numbers after the legacy reserved minor device numbers.

If using **modprobe** to load (or auto-load) the **comedi** kernel
module, the options can be specified in the
*`/etc/modprobe.d/comedi.conf`* file (or the *`/etc/modprobe.conf`* file
on older systems, containing an **`options comedi`** line such as:

```
options comedi comedi_num_legacy_minors=4
```

### Configure Modules To Load At System Boot

The _/etc/modules-load.d/\*.conf_ files list the names of modules to be
loaded at system boot.  The module names are separated by newlines.
Empty lines and comment lines (whose first non-whitespace character is
**`;`** or **`#`**) are ignored.  This may be useful for loading modules
to support legacy **Comedi** device types at system boot.  This is not
usually required for PCI and USB device drivers that will be loaded
automatically when a matching device is detected.

You may also want to run a script during system boot to configure the
legacy devices using the **comedi_config** command from the
**Comedilib** project.

### UDEV rules

By default, device files in */dev* that are dynamically created by
**udev** will be owned by user *root* and group *root*, and will only be
readable and writable by the owner *root* (mode `0600`).  This can be
changed by creating UDEV rules to change the permissions.

A new UDEV rules file can be created in the */etc/udev/rules.d*
directory, for example */etc/udev/rules.d/90-comedi.rules*.  To make the
*/dev/comedi\** files readable and writable by everyone, use the
following UDEV rule:

```
KERNEL=="comedi[0-9]*", MODE=0666
```

However, it would be more secure limit access to the user *root* and
members of a supplementary system group to which legitimate users can be
added.  **Debian GNU/Linux** suggests creating the group called
*iocard*.  Debian's **libcomedi0** package (to install **Comedilib**)
creates the system group *iocard* and installs the following UDEV rule
(in */lib/udev/rules.d/90-comedi.rules*):

```
KERNEL=="comedi[0-9]*", MODE=0660, GROUP="iocard"
```

The **groupadd** command may be used to create the group if necessary,
for example (using **sudo** to run it):

```
sudo groupadd -r iocard
```

The **`-r`** or **`--system`** option creates the group within the range
of group ID numbers reserved for system groups, but the any group ID
number will do.

The **gpasswd** command (or various other commands) may be used to add
an existing user to the group, for example (using **sudo** to run it):

```
sudo gpasswd -a francis iocard
```

The user will need to log in again to inherit the group permissions.

### Static Device Files

Originally, */dev* was used as part of a normal filesystem (rather than
a mountpoint for a virtual filesystem managed by **udev**), and static
device files had to be created using the **mknod** program.  There may
be some modern systems where this is still the case for some reason.

If the device files need to be created statically, note that the
character special major device number 98 has been reserved for
**Comedi** devices.  Up to 16 special character device files may be
created with major device number 98 and minor device numbers 0 to 15.
These can be created by running **`sudo make dev`** from the **Comedi**
build directory, or by running the following shell command:

```
sudo for i in $(seq 0 15); do mknod -m 666 /dev/comedi$i c 98 $i; done
```

That will make the device files accessible to everyone. To make them
accessible only to the owning user and group, change **`-m 666`** to
**`-m 660`** in the above command, or change the mode afterwards using
__`chmod 660 /dev/comedi*`__ and use the **chgrp** command to change the
group owner of the files, for example: __`chgrp iocard /dev/comedi*`__.

### Comedilib

Now would be a good time to compile and install **Comedilib**.
**Comedi** and **Comedilib** are completely independent, so it doesn't
matter which is installed first.

## Running Comedi

To use **Comedi**, the low-level hardware driver module and its
dependencies, including the core **comedi** module need to be loaded
into the kernel.  This is done automatically for some devices that are
detected during system boot (mostly PCI and USB devices), but for legacy
"Comedi" devices, this is done manually by using the **modprobe**
command, similar to:

```
sudo modprobe your_driver
```

where **`your_driver`** is the name of the **Comedi** low-level hardware
driver module.  See section *[Configure Modules To Load At System
Boot][modload]* to do this automatically.

In order to configure a **Comedi** device file to use a particular type
of **Comedi** legacy device (specified by a "board name" that is
supported by the driver module, and may or may not be the same as the
module's "driver name"), you need to use the command **comedi_config**,
which is built from the **Comedilib** distribution.  The
**comedi_config** command is invoked using:

```
sudo comedi_config /dev/comediN BOARDNAME OPTION-LIST
```

where **`/dev/comediN`** is the pathname of the device file,
**`BOARDNAME`** is the device's *board name* (which may or may not be
the same as the module name), and **`OPTION-LIST`** is a comma-separated
list of integer values to set information required by the device, for
example its I/O base address and IRQ number.  These options are specific
to the particular board name and are documented in the
*Documentation/comedi/drivers.txt* file in the **Comedi** source
directory (for sources from a release tarball), or the **Comedi** build
directory (when building sources from the **Git** repository), and in
the **Comedilib** reference manual.

The following commands are examples:

```
sudo comedi_config /dev/comedi0 dt2821 0x240,3
sudo comedi_config /dev/comedi1 ni_atmio 0x260,4
sudo comedi_config /dev/comedi2 dt2817 0x228
```

You may wish to use a script to run the commands (without the **sudo**
part) during system boot.

Try running **`man comedi_config`** for information on how to use this
utility.  Scripts have been written for a few of the drivers with very
complicated option lists.  These may be found in **Comedilib**'s *etc*
directory.

The **Comedilib** project includes a program **comedi_test** to run some
basic tests on the **Comedi** device, and a program
**comedi_board_info** to get information about a configured **Comedi**
device.  The **Comedilib** project also includes source code for various
demonstration programs.

The */proc/comedi* file includes information about configured **Comedi**
devices and loaded **Comedi** drivers.

## Generating Unofficial Releases

An unofficial release tarball is useful when using **[Akmods][]** or
**[DKMS][]**. It may also be useful when building on a different Linux
system that does not have **[Autoconf]** and **[Automake]** installed.

An unofficial release tarball can be generated by running this command
in the **Comedi** build directory after configuring the
**Comedi** build:

```
make dist-gzip
```

> [!NOTE]
> The configuration needs to complete successfully, which requires it to
> be configured to build for a specific kernel, even though we are not
> building **Comedi** modules at this stage.  This limitation may be
> fixed in the future.

The tarball file will be generated in the build directory and will have
a weird name something like *comedi-0.7.76.1.373-58cbb.tar.gz*.  The
files within the tarball will be in a sub-directory with the same name,
but without the *.tar.gz* extension.

The files in the tarball will be listed as belonging to the current user
ID and group ID.  It may be better to use the **fakeroot** command
(installed by the **fakeroot** package) to make the files in the tarball
owned by user ID 0 (*root*) and group ID 0 (*root*).  For example:

```
fakeroot make dist-gzip
```

### Installing From The Tarball

To do a manual build using the sources from the unofficial release
tarball, change to a suitable directory and unpack the tarball using the
**tar** command, for example:

```
tar zxvf /path/to/comedi-0.7.76.1.373-58cbb.tar.gz
```

The sources will be unpacked in the current directory, which will create
a directory with a similar name, such as *comedi-0.7.76.1.373-58cbb*.
Follow the instructions from section *[In-Tree And Out-Of-Tree
Builds][builddir]* onwards, using the created directory as the
**Comedi** top-level source directory.

## Automatic Installation

On several Linux distributions, it is possible to configure the
operating system to rebuild and install external kernel modules from
source automatically when a new Linux kernel is installed (possibly
during the next system reboot for **Akmods**).  It can also be
configured to digitally sign these modules with a local key that can be
enrolled for use with **Secure Boot**.

On modern Red Hat distributions (including Fedora Linux, RHEL, CentOS,
and derivatives such as AlmaLinux and Rocky Linux), the **[Akmods][]**
system is used.  See the section *[Installation Via Akmods][akmodsins]*
for details.

On non Red Hat distributions (including Debian, Ubuntu, Arch Linux,
amongst others), the **[Dynamic Kernel Module System (DKMS)][dkms]**
system is used.  See the section *[Installation Via DKMS][dkmsins]* for
details.

When using **Akmods** or **DKMS**, you need to supply a release of the
**Comedi** sources to be built.  See the section *[Generating Unofficial
Releases][genrel]* for details of generating an unofficial release
tarball suitable for use with **Akmods** or **DKMS**.

### Installation Via Akmods

**[Akmods][]** is used on modern Red Hat distributions to manage the
building and installation of external, out-of-tree Linux kernel modules
for new kernels installed by the system.

We differentiate between the "build" system used to generate the
_akmod-\*.rpm_ packages, and the "target" system where those packages
will be installed (although there will be some automatic building work
on the target system to build the actual Linux kernel modules).  The
build system may also be the target system.

#### Installing The akmods Package

The **akmods** package needs to be installed on both the build system
and the target system (if different).

If planning to install **akmods** on a target system without Internet
access, RPMs of the **akmods** and **kmodtool** packages for the distro
need to be obtained.  They have several dependencies, but those should
be installable from official distribution ISOs.  For Fedora Linux,
browse the **[Fedora Packages][fedpack]** website for the required
packages.  For RHEL, CentOS, AlmaLinux, and Rocky Linux, search for
"Available Packages" on the **[Extra Packages for Enterprise Linux
(EPEL)][epel]** website for the required packages.  The downloaded
**kmodtool** and **akmods** RPM packages can then be installed using the
**dnf install** command:

```
sudo dnf install kmodtool-*.rpm akmods-*.rpm
```

(Substitute the full RPM filenames for __`*`__ if necessary.)

It is easier to install the **akmods** package and its dependencies from
online repositories, if possible.  On the build system, the online
repositories need to be enabled anyway, so they might as well be used to
install the **akmods** package.  For Fedora Linux, the required
repositories are already enabled, but for RHEL, CentOS, AlmaLinux, and
Rocky Linux, the **EPEL** repository needs to be enabled.  See the
**[EPEL][]** website for reference if the following commands are
out-of-date.

For RHEL, the following commands are used to enable the **EPEL**
repository:

```
sudo subscription-manager repos --enable codeready-builder-for-rhel-$(rpm -E %rhel)-$(arch)-rpms
sudo dnf install https://dl.fedoraproject.org/pub/epel/epel-release-latest-$(rpm -E %rhel).noarch.rpm
```

For CentOS, AlmaLinux, and Rocky Linux, the following commands are used
to enable the **EPEL** repository:

```
sudo dnf config-manager --set-enabled crb
sudo dnf install https://dl.fedoraproject.org/pub/epel/epel-release-latest-$(rpm -E %rhel).noarch.rpm
sudo dnf install https://dl.fedoraproject.org/pub/epel/epel-next-release-latest-$(rpm -E %rhel).noarch.rpm
```

> [!NOTE]
> The **`$(rpm -E %rhel)`** part can be replaced with an explicit **EL**
> number, such as __`9`__ for RHEL 9, Rocky Linux 9, etc.

Once the **EPEL** repository has been enabled (not needed for Fedora
Linux), the **akmods** package and its dependencies (including
**kmodtool**) can be installed with the following command:

```
sudo dnf install akmods
```

#### Enrolling A Signing Key For Akmods

When **akmods** is installed, or when the **akmods** Systemd service is
first run, the **akmods** package creates a certificate and signing key
pair, but it does not automatically enroll the public key as a "Machine
Owner Key" ("MOK").  It is possible to replace the automatically created
certificate and signing key pair with your own version.  See the
*/usr/share/doc/akmods/README.secureboot* file installed by the
**akmods** package for details on creating your own certificate and key
pair and for enrolling the (original or replacement) public key as a
MOK.

> [!NOTE]
> The MOK enrollment process involves the use of a temporary password
> that needs to be repeated during the next system boot, and it may be
> using a default US English keyboard layout at that time.  If you do
> not know the positions of the characters on the US English keyboard
> layout, use something fairly universal (such as a string of ASCII
> digit characters 0 to 9) for the temporary password.

That only needs to be done on the target system when UEFI secure boot is
enabled and/or the kernel is enforcing valid signature checks during
module loading.  The signing key is not needed to generate the packages
on the build system.

#### Setting Up The Build Environment

On the build system, the **[RPM Fusion][rpmfusion]** repository needs to
be enabled so that the
**buildsys-build-rpmfusion-kerneldevpkgs-current** package can be
installed.  See the instructions at
<https://rpmfusion.org/Configuration> for reference.  There are **free**
and **nonfree** repositories, but we only need to set up the **free**
repository.

For Fedora, use the following command to set up the **RPM Fusion free**
repository:

```
sudo dnf install https://mirrors.rpmfusion.org/free/fedora/rpmfusion-free-release-$(rpm -E %fedora).noarch.rpm
```

For RHEL, CentOS, AlmaLinux and Rocky Linux, the **EPEL** repository
needs to be enabled as described in the section *[Installing The akmods
Package][akmodspkgins]*.  Then use the following command to set up the
**RPM Fusion free** repository:

```
sudo dnf install https://mirrors.rpmfusion.org/free/el/rpmfusion-free-release-$(rpm -E %rhel).noarch.rpm
```

Once the **RPM Fusion free** repository has been enabled, install the
**buildsys-build-rpmfusion-kerneldevpkgs-current** package:

```
sudo dnf install buildsys-build-rpmfusion-kerneldevpkgs-current
```

(The **rpmdevtools** and **akmods** packages are also required, but
**akmods** was installed in a previous step, and **rpmdevtools** is one
of its dependencies.)

Now the *rpmbuild* need to be created in the builder's home directory.

Run the **`rpmdev-setuptree`** command to set up the *rpmbuild*
directories.  This will create the *~/rpmbuild* directory (where *~*
represents the builder's home directory), containing subdirectories
*BUILD*, *RPMS*, *SOURCES*, *SPECS*, and *SRPMS*. It also creates a
*~/.rpmmacros* file.

The *~/.rpmmacros* file can be edited to set a "provider" tag that will
appear in the names of the generated RPM files.  If used, the provider
tag should start with a ".".  For example, to set the provider tag to
".mytag", edit the *~/.rpmmacros* file and add the following line to the
end of the file:

```
%prov .mytag
```

#### Building The Comedi RPM Files For akmods

Once the build environment has been set up, follow these steps on the
build system to generate the RPM files to be installed on the target
system:

> [!NOTE]
> In the following steps:
>
> * The **`~`** shell expansion character represents the builder's home
>   directory.
>
> * **`${VER_TGZ}`** represents the **Comedi** version in the name
>   of the tarball, such as **`0.7.76.1.373-58cbb`**.
> * **`${VER_RPM}`** represents the **Comedi** version in the RPM files,
>   which is the same as **`${VER_TGZ}`** with hyphens changed to
>   underscores, such as **`0.7.76.1.373_58cbb`**.
> * **`${DIST}`** represents the output from the **`rpm -E '%{?dist}'`**
>   command, such as **.fc42** for Fedora 42.
> * **`${PROV}`** represents the provider tag (if configured) from the
>   output of the **`rpm -E '%{?prov}'`** command.
> * **`${ARCH}`** represents the architecture of the target machine as
>   output by the **`arch`** or **`uname -m`** command on the target
>   machine.
> * **`${KVER}`** represents the kernel version of an installed
>   **kernel-devel** package.

1.  Copy the (unofficial) release tarball file to the
    *~/rpmbuild/SOURCES* directory:

    ```
    cp /path/to/comedi-${VER_TGZ}.tar.gz ~/rpmbuild/SOURCES
    ```

    Note that the file will be automatically deleted in the next step, so
    keep a copy!

2.  Convert the tarball to an SRPM file, which will delete the tarball in
    *~/rpmbuild/SOURCES* and create the
    *comedi-${VER_RPM}-1${DIST}${PROV}.src.rpm* file:

    ```
    rpmbuild -ts ~/rpmbuild/SOURCES/comedi-${VER_TGZ}.tar.gz
    ```

3.  Build the arch-specific RPM files from the SRPM file:

    ```
    rpmbuild --rebuild --target ${ARCH} ~/rpmbuild/SRPMS/comedi-${VER_RPM}-1${DIST}${PROV}.src.rpm
    ```

    The following generated RPM files will be placed in the
    *~/rpmbuild/RPMS/${ARCH}* directory:

    * *akmod-comedi-${VER_RPM}-1${DIST}${PROV}.${ARCH}.rpm*
    * *kmod-comedi-${VER_RPM}-1${DIST}${PROV}.${ARCH}.rpm*
    * Zero or more kernel-specific
     *kmod-comedi-${KVER}-${VER_RPM}-1${DIST}${PROV}.${ARCH}.rpm* files.

    The *akmod-comedi-${VER_RPM}-1${DIST}${PROV}.${ARCH}.rpm* file is
    required to be installed on the target system.  The other RPM files
    are not required.

4. Build the non-arch-specific RPM files from the SRPM file:

    ```
    rpmbuild --rebuild --target noarch ~/rpmbuild/SRPMS/comedi-${VER_RPM}-1${DIST}${PROV}.src.rpm
    ```

    The following generated RPM files will be placed in the
    *~/rpmbuild/RPMS/noarch* directory:

    * *comedi-common-${VER_RPM}-1${DIST}${PROV}.noarch.rpm*

    The *comedi-common-${VER_RPM}-1${DIST}${PROV}.noarch.rpm* file is
    required to be installed on the target system.

#### Installing The Comedi RPM Files On The Target System

The following files need to installed on the target system:

* *comedi-common-${VER_RPM}-1${DIST}${PROV}.noarch.rpm*
* *akmod-comedi-${VER_RPM}-1${DIST}${PROV}.${ARCH}.rpm*

where:

* *${VER_RPM}* is the RPM package version, such as *0.7.76.1.373_58cbb*
* *${DIST}* is the Linux distribution code, such as *.fc42* or *.el9*
* *${PROV}* is an optional "provider" tag, such as *.mytag*
* *${ARCH}* is the machine architecture, such as *x86_64*

For example:

* *comedi-common-0.7.76.1.373_58cbb-1.fc42.noarch.rpm*
* *akmod-comedi-0.7.76.1.373_58cbb-1.fc42.x86_64.rpm*

> [!WARNING]
> Before installing the **akmod-comedi** package, check for any
> installed kernel-specific **kmod-comedi** packages and remove them,
> otherwise there will be problems.  The kernel-specific **kmod-comedi**
> packages are built and installed by the **akmods** **systemd**
> service, but the installation phase will fail due to file conflicts
> with the currently installed package.
>
> The old **kmod-comedi** packages can be removed using the command:
>
> ```
> sudo dnf remove 'kmod-comedi-*_*_*'
> ```

Example commands to install the **comedi-common** and **akmod-comedi**
packages from RPM files in the current directory:

```
sudo dnf install comedi-common-0.7.76.1.373_58cbb-1.fc42.noarch.rpm
sudo dnf akmod-comedi-0.7.76.1.373_58cbb-1.fc42.x86_64.rpm
```

(Adjust those commands to use the actual version being installed.)

The **Comedi** kernel modules will be built, signed, and installed on
the target system by the **akmods** **[systemd][]** service, which
normally runs during system boot, but may also be triggered to
run in the background after an __akmod-__ package is installed.

> [!NOTE]
> As **Comedi** consists of many kernel modules, it may take a few
> minutes for **akmods** to build them.

If the **akmods** **systemd** service was not triggered automatically,
and you do not want to wait for a reboot, the service can be started
manually:

```
sudo systemctl start akmods.service
```

> [!NOTE]
> **akmods** will generate kernel-specific **kmod-comedi** packages in
> the */var/cache/akmods/comedi* directory, along with some _\*.log_
> files on success, or _\*.failed.log_ files on failure.  Check the
> latest _\*.failed.log_ file for problems.

### Installation Via DKMS

TBA




[akmods]: https://rpmfusion.org/Packaging/KernelModules/Akmods "Akmods"
[autoconf]: https://www.gnu.org/software/autoconf/ "Autoconf"
[automake]: https://www.gnu.org/software/automake/ "Automake"
[autoins]: #automatic-installation "Automatic installation"
[comedi]: https://www.comedi.org/ "Comedi"
[mail]: README.md#mailing-list "Comedi mailing list"
[config]: #configuration "Configuration"
[modload]: #configure-modules-to-load-at-system-boot "Configure modules to load at system boot"
[snap]: #downloading-a-snapshot-of-the-sources "Downloading a snapshot"
[dkms]: https://github.com/dell/dkms "Dynamic Kernel Module System (DKMS)"
[epel]: https://docs.fedoraproject.org/en-US/epel/ "Extra Packages for Enterprise Linux (EPEL)"
[fedpack]: https://packages.fedoraproject.org/ "Fedora Packages"
[genrel]: #generating-unofficial-releases "Generating unofficial releases"
[git]: https://git-scm.com/ "Git web site"
[builddir]: #in-tree-and-out-of-tree-builds "In-tree and out-of-tree builds"
[akmodspkgins]: #installing-the-akmods-package "Installing the akmods package"
[akmodsins]: #installation-via-akmods "Installation via Akmods"
[dkmsins]: #installation-via-dkms "Installation via DKMS"
[linux]: https://www.kernel.org/ "Linux kernel"
[prep]: #preparing-the-sources-from-git "Preparing the sources"
[readme.md]: README.md "READ ME"
[rpmfusion]: https://rpmfusion.org/ "RPM Fusion"
[rtai]: https://www.rtai.org/ "RTAI - Real Time Application Interface"
[systemd]: https://systemd.io/ "Systemd"
