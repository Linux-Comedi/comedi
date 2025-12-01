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
> before **Comedi** can be configured to be built.  See the section
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
*Linux-Comedi-comedi-v0.7.76.1-395-g250ba89.tar.gz*, depending on the
state of the **Comedi** **Git** repository at the time.  This can be
unpacked using the **tar** command, and will create a sub-directory with
a similar name (but without the *.tar.gz* extension) containing the
snapshot of the sources.  For example:

```
tar zxvf Linux-Comedi-comedi-v0.7.76.1-395-g250ba89.tar.gz
```

Change directory to that sub-directory for further operations on the
sources.  For example:

```
cd Linux-Comedi-comedi-v0.7.76.1-395-g250ba89
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

If there is an error from **autoreconf** about failing to run
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
**Linux** kernel build directory.  If you are building for a **Linux**
kernel patched for **[RTAI][]** or **RTLinux**, the **`--with-rtaidir`**
or **`--with-rtlinuxdir`** options allow you to specify the location of
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
> keys, but that is beyond the scope of this section.  Some of the
> details depend on the Linux distribution.  The automatic installation
> methods described in section *[Automatic Installation][autoins]* have
> the option to sign the kernel modules that they build.

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
detected during system boot (mostly PCI and USB devices), but for
"legacy" device types (such as ISA cards), this is done manually by
using the **modprobe** command, similar to:

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
**[DKMS][]**. It may also be useful when building on a different
**Linux** system that does not have **[Autoconf]** and **[Automake]**
installed.

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
a weird name something like *comedi-0.7.76.1.395-250b.tar.gz*.  The
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
tar zxvf /path/to/comedi-0.7.76.1.395-250b.tar.gz
```

The sources will be unpacked in the current directory, which will create
a directory with a similar name, such as *comedi-0.7.76.1.395-250b*.
Follow the instructions from section *[In-Tree And Out-Of-Tree
Builds][builddir]* onwards, using the created directory as the
**Comedi** top-level source directory.

## Automatic Installation

On several **Linux** distributions, it is possible to configure the
operating system to rebuild and install external kernel modules from
source automatically when a new **Linux** kernel is installed (possibly
during the next system reboot for **Akmods**).  It can also be
configured to digitally sign these modules with a local key that can be
enrolled for use with **Secure Boot**.

On modern Red Hat distributions (including **Fedora Linux**, **RHEL**,
**CentOS**, and derivatives such as **AlmaLinux** and **Rocky Linux**),
the **[Akmods][]** system is used.  See the section *[Installation Via
Akmods][akmodsins]* for details.

On non Red Hat distributions (including **Debian**, **Ubuntu**, **Arch
Linux**, amongst others), the **[Dynamic Kernel Module System
(DKMS)][dkms]** system is used.  See the section *[Installation Via
DKMS][dkmsins]* for details.

When using **Akmods** or **DKMS**, you need to supply a release of the
**Comedi** sources to be built.  See the section *[Generating Unofficial
Releases][genrel]* for details of generating an unofficial release
tarball suitable for use with **Akmods** or **DKMS**.

### Installation Via Akmods

**[Akmods][]** is used on modern Red Hat distributions to manage the
building and installation of external, out-of-tree **Linux** kernel
modules for new  and existing kernels installed by the system.

We differentiate between the "build" system used to generate the
_akmod-\*.rpm_ packages, and the "target" system where those packages
will be installed (although there will be some automatic building work
on the target system to build the actual **Linux** kernel modules).  The
build system may also be the target system.

#### Installing The akmods Package

The **akmods** package needs to be installed on both the build system
and the target system (if different).

If planning to install **akmods** on a target system without Internet
access, RPMs of the **akmods** and **kmodtool** packages for the
**Linux** distribution need to be obtained.  They have several
dependencies, but those should be installable from official distribution
ISOs.  For **Fedora Linux**, browse the **[Fedora Packages][fedpack]**
website for the required packages.  For **RHEL**, **CentOS**,
**AlmaLinux**, and **Rocky Linux**, search for "Available Packages" on
the **[Extra Packages for Enterprise Linux (EPEL)][epel]** website for
the required packages.  The downloaded **kmodtool** and **akmods** RPM
packages can then be installed using the **dnf install** command:

```
sudo dnf install kmodtool-*.rpm akmods-*.rpm
```

(Substitute the full RPM filenames for __`*`__ if necessary.)

It is easier to install the **akmods** package and its dependencies from
online repositories, if possible.  On the build system, the online
repositories need to be enabled anyway, so they might as well be used to
install the **akmods** package.  For **Fedora Linux**, the required
repositories are already enabled, but for **RHEL**, **CentOS**,
**AlmaLinux**, and **Rocky Linux**, the **EPEL** repository needs to be
enabled.  See the **[EPEL][]** website for reference if the following
commands are out-of-date.

For **RHEL**, the following commands are used to enable the **EPEL**
repository:

```
sudo subscription-manager repos --enable codeready-builder-for-rhel-$(rpm -E %rhel)-$(arch)-rpms
sudo dnf install https://dl.fedoraproject.org/pub/epel/epel-release-latest-$(rpm -E %rhel).noarch.rpm
```

For **CentOS**, **AlmaLinux**, and **Rocky Linux**, the following
commands are used to enable the **EPEL** repository:

```
sudo dnf config-manager --set-enabled crb
sudo dnf install https://dl.fedoraproject.org/pub/epel/epel-release-latest-$(rpm -E %rhel).noarch.rpm
sudo dnf install https://dl.fedoraproject.org/pub/epel/epel-next-release-latest-$(rpm -E %rhel).noarch.rpm
```

> [!NOTE]
> The **`$(rpm -E %rhel)`** part can be replaced with an explicit **EL**
> number, such as __`9`__ for RHEL 9, Rocky Linux 9, etc.

Once the **EPEL** repository has been enabled (not needed for **Fedora
Linux**), the **akmods** package and its dependencies (including
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

For **Fedora Linux**, use the following command to set up the **RPM Fusion
free** repository:

```
sudo dnf install https://mirrors.rpmfusion.org/free/fedora/rpmfusion-free-release-$(rpm -E %fedora).noarch.rpm
```

For **RHEL**, **CentOS**, **AlmaLinux** and **Rocky Linux**, the
**EPEL** repository needs to be enabled as described in the section
*[Installing The akmods Package][akmodspkgins]*.  Then use the following
command to set up the **RPM Fusion free** repository:

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

Now the *rpmbuild* directory need to be created in the builder's home
directory.

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
>   of the tarball, such as **`0.7.76.1.395-250b`**.
> * **`${VER_RPM}`** represents the **Comedi** version in the RPM files,
>   which is the same as **`${VER_TGZ}`** with hyphens changed to
>   underscores, such as **`0.7.76.1.395_250b`**.
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

* *${VER_RPM}* is the RPM package version, such as *0.7.76.1.395_250b*
* *${DIST}* is the **Linux** distribution code, such as *.fc42* or
  *.el9*
* *${PROV}* is an optional "provider" tag, such as *.mytag*
* *${ARCH}* is the machine architecture, such as *x86_64*

For example:

* *comedi-common-0.7.76.1.395_250b-1.fc42.noarch.rpm*
* *akmod-comedi-0.7.76.1.395_250b-1.fc42.x86_64.rpm*

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
sudo dnf install comedi-common-0.7.76.1.395_250b-1.fc42.noarch.rpm
sudo dnf install akmod-comedi-0.7.76.1.395_250b-1.fc42.x86_64.rpm
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

**[DKMS][]** can be used on several distributions (including **Arch
Linux**, **Debian GNU/Linux**, **Linux Mint**, **openSUSE** (and
**SLES** with the addition of an **openSUSE** package repository), and
**Ubuntu**, amongst others) to manage the building and installation of
external, out-of-tree **Linux** kernel modules for new and existing
kernels installed by the system.  It can also sign the modules it builds
for use with **Secure Boot**.

#### Installing DKMS

##### Installing DKMS On Arch Linux

On **Arch Linux**, use the following commands to install the **dkms**
package:

```
sudo pacman -Sy
sudo pacman -S dkms
```

It is also be necessary to install one of the **linux-headers**,
**linux-hardened-headers**, **linux-lts-headers**, or
**linux-rt-headers** packages depending on which flavor of the **linux**
package is installed.  Check with the command **`pacman -Qo
{/usr,}/lib/modules`**, which will show the names and versions of
packages that have installed **Linux** kernel modules or headers.
(There may be several packages installed for different versions of the
kernel and kernel headers packages.)

##### Installing DKMS On Debian, Linux Mint, and Ubuntu

On Debian-based distributions, use the following commands to install the
**dkms** package:

```
sudo apt-get update
sudo apt-get install dkms
```

It may also be necessary to install a **linux-headers** or
**linux-headers-generic** virtual package to add support for building
external kernel modules.  The choice depends on the virtual package that
was selected to install the **linux-image** flavor.  Check the output of
the **`dpkg-query -l 'linux-image-*'`** command to determine that.
Alternatively, the command **`dpkg-query -S {/usr,}/lib/modules`** will
show the names and versions of all packages that have installed
**Linux** kernel modules or headers.  (There may be several packages
installed for different versions of the kernel and kernel headers
packages.)

On some distributions, **dkms** and/or the appropriate **linux-headers**
package may have been already installed as part of the initial system
installation.

##### Installing DKMS on openSUSE

On **openSUSE**, use the following commands to install the **dkms**
package:

```
sudo zypper refresh
sudo zypper install dkms
```

It may install a **kernel-syms** package as a dependency for building
external kernel modules.

#### Setting Up Module Signing Support

On systems with kernels that enforce checks for valid signatures during
module loading for secure boot, the kernel modules built by **DKMS**
need to be signed with a Machine Owner Key (MOK) that has a matching
certificate enrolled in the system.

The **dkms** command can generate a certificate key automatically, but
depending on the version of the **dkms** package, and the operating
system, the automatically generated key might be suitable.  For example,
**openSUSE Leap 15.6** requires the certificate to have "Code Signing"
extended key usage, but its **dkms** package version is too old to
generate a certificate with this usage.  So it may be necessary to
generate the certificate and key manually.

##### Setting Up Module Signing Support On Arch Linux

**Arch Linux** installations do not use **Secure Boot** by default.  In
particular, **Linux** kernels booted securely will normally be signed
with a Machine Owner Key (MOK) that has already been enrolled for use by
**Secure Boot**.  Since the certificate and signing key that was used to
sign the kernel image should be already on the system somewhere, it may
be tempting to configure **DKMS** to use the same certificate and
signing key to sign the modules that it builds.  Alternatively, a
separate signing key and certificate may be used for **DKMS**.

> [!NOTE]
> At the time of writing, **Arch Linux** kernels (up to at least kernel
> version 6.15) are not configured with the *`CONFIG_IMA`* and
> *`CONFIG_IMA_SECURE_AND_OR_TRUSTED_BOOT`* options enabled, which means
> there is no point signing external kernel modules with a MOK because
> the kernel will not trust the signature anyway.  Fortunately, that
> also means the kernel will not enforce checks for valid module
> signatures.  Perhaps this will change in the future, but for now it
> does not matter if the signing key and certificate that **DKMS** uses
> to sign the modules it builds is enrolled for use with **Secure Boot**
> or not.

###### Using An Existing MOK On Arch Linux

If a kernel installed by **Arch Linux** are already being signed by with
a MOK (for example, by a post installation hook script), then **DKMS**
can be configured to use the same key pair to sign the modules it
builds.  This is done by setting the *`mok_signing_key`* and
*`mok_certificate`* values in the _/etc/dkms/framework.conf_ or
_/etc/dkms/framework.conf.d/\*.conf_ files.  They should be set to the
pathnames of the existing signing key and certificate.

> [!NOTE]
> There may be two files for the certificate (in addition to the key
> file) in different formats, a binary format (DER) and an ASCII-armored
> format (PEM).  The *`mok_certificate`* value should be set to the
> pathname of the binary format (DER) certificate.

###### Using A New MOK On Arch Linux

If a separate MOK is to be used for **DKMS**, you may allow the **dkms**
to generate its own signing key and certificate when it builds a module
package for the first time.  For **dkms** version 3.1 or later, these
can be generated before building the first module package by using the
command *`sudo dkms generate_mok`*.  If the signing key and certificate
already exist, they will not be regenerated.

Alternatively, **DKMS** can be configured to use a manually generated
signing key and certificate (in DER format) by setting the
*`mok_signing_key`* and *`mok_certificate`* variables in the
_/etc/dkms/framework.conf_ or _/etc/dkms/framework.conf.d/\*.conf_ files
to the pathnames of the signing key and certificate files.  If those
variables are left unset, the signing key and certificate could be
copied to the default locations _/var/lib/dkms/mok.key_ and
_/var/lib/dkms/mok.pub_.

The MOK certificate will need to be enrolled for use with **Secure
Boot**, but that can be done later.  See section *[Enrolling The DKMS
Signing Certificate][dkmsenroll]* for details.

> [!NOTE]
> Actually, at the time of writing, for **Arch Linux** kernels up to at
> least kernel version 6.15, the MOK certificate does not need to be
> enrolled, because the kernel is not configured to trust the MOK
> certificates, so will treat them as invalid.  That is OK as long as
> the *`module.sig_enforce`* kernel command-line parameter is not
> enabled.  The kernel will warn that the module has an invalid
> signature, but will load it anyway.  This might change in a future
> **Arch Linux** kernel.

##### Setting Up Module Signing Support On Debian

On **Debian**, the default locations of the **dkms** signing key and
certificate are _/var/lib/dkms/mok.key_ and _/var/lib/dkms/mok.pub_.
They can be overridden by setting the *`mok_signing_key`* and
*`mok_certificate`* variables in the _/etc/dkms/framework.conf_ or
_/etc/dkms/framework.conf.d/\*.conf_ files to the pathnames of the
signing key and certificate files.  The **dkms** command will generate a
signing key and certificate file automatically if they do not exist when
building a module package.  For **dkms** version 3.1 or later, these can
be generated before building the first module package by using the
command *`sudo dkms generate_mok`*.  If the signing key and certificate
already exist, they will not be regenerated.  Alternatively, a
user-supplied signing key and certificate (in DER format) may be
supplied.

The MOK certificate will need to be enrolled for use with **Secure
Boot**, but that can be done later.  See section *[Enrolling The DKMS
Signing Certificate][dkmsenroll]* for details.

##### Setting Up Module Signing Support On Linux Mint

For **Linux Mint Debian Edition (LMDE)**, follow the instructions for
**Debian** in section *[Setting Up Module Signing Support On
Debian][dkmsmokdeb]*.

For regular **Linux Mint** editions, if you opted for installation of
multimedia drivers during system installation, the system may have
already installed a MOK signing key and certificate in
_/var/lib/shim-signed/mok/MOK.priv_ (signing key) and
_/var/lib/shim-signed/mok/MOV.der_ (certificate).  It is possible to
configure **dkms** to use those certificates by editing the
_/etc/dkms/framework.conf_ or _/etc/dkms/framework.conf.d/\*.conf_ files
to set the *`mok_signing_key`* and *`mok_certificate`* variables to the
pathnames of the existing signing key and certificate.  Without those
changes to the **DKMS** configuration files, **dkms** will ignore the
existing signing key and generate a new signing key and certificate that
will need to be enrolled separately.

If **DKMS** has not been configured to use an existing signing key and
certificate, follow the instructions for **Debian** in section *[Setting
Up Module Signing Support On Debian][dkmsmokdeb]*.

##### Setting Up Module Signing Support On openSUSE

On **openSUSE**, the **dkms** command can generate a signing key and
certificate when building a module package for the first time, but the
certificate may not be suitable.  On **openSUSE 15.6**, the kernel
requires external modules to be signed with a key that includes "Code
Signing" external key usage, but the version of **dkms** from the
package repository is too old to generate such a key automatically
(which requires **dkms** version 1.1.7 or later).

To create a signing key and certificate for **openSUSE** kernels that
require a "Code Signing" certificate, run the following command to
generate a certificate pair:

```
openssl req -new -x509 -nodes -days 36500 -subj "/CN=DKMS module signing key" \
-newkey rsa:2048 -keyout mok.key -addext "extendedKeyUsage=codeSigning" \
-outform DER -out mok.pub
```

Note that the above command requires **OpenSSL** version 1.1.1 or later
to add the extended key usage to the signing key.  For older versions of
**OpenSSL**, omit the **`-addext "extendedKeyUsage=codeSigning"`**
option from the above command.  (Hopefully, the kernel will not require
a "Code Signing" certificate on **openSUSE** releases that have older
versions of the **openssl** package, otherwise the signing key and
certificate would need to be generated in an environment with a more
up-to-date version of **OpenSSL**.)

After generating the signing key and certificate, they need to be copied
to the */var/lib/dkms* directory:

```
sudo cp mok.key mok.pub /var/lib/dkms
```

> [!NOTE]
> The location of the signing key and certificate files can be
> overridden by the values of the *`mok_signing_key`* and
> *`mok_certificate`* variables in _/etc/dkms/framework.conf_ and
> _/etc/dkms/framework.conf.d/\*.conf_ files.

The MOK certificate will need to be enrolled for use with **Secure
Boot**, but that can be done later.  See section *[Enrolling The DKMS
Signing Certificate][dkmsenroll]* for details.

##### Setting Up Module Signing Support On Ubuntu

On **Ubuntu**, the default locations of the **dkms** signing key and
certificate are _/var/lib/shim-signed/mok/MOK.priv_ and
_/var/lib/shim-signed/mok/MOK.der_.  They can be overridden by setting
the *`mok_signing_key`* and *`mok_certificate`* variables in the
_/etc/dkms/framework.conf_ or _/etc/dkms/framework.conf.d/\*.conf_ files
to the pathnames of the signing key and certificate files.  The **dkms**
command will generate a signing key and certificate file automatically
if they do not exist when building a module package.  For **dkms**
version 3.1 or later, these can be generated before building the first
module package by using the command *`sudo dkms generate_mok`*.  If the
signing key and certificate already exist, they will not be regenerated.
Alternatively, a user-supplied signing key and certificate (in DER
format) may be supplied.

The MOK certificate will need to be enrolled for use with **Secure
Boot**, but that can be done later.  See section *[Enrolling The DKMS
Signing Certificate][dkmsenroll]* for details.

#### Adding Comedi To DKMS

After installing **DKMS**, and setting up module signing support (if not
being done by **dkms** automatically), **Comedi** can be added to
**DKMS** as a new module package.  **DKMS** allows multiple versions of
a module package to be present in the system, so module packages are
represented in **DKMS** as a *package-name/package-version* pair
(although the **DKMS** manual refers to them as a
*module/module-version* pair).  The package name and package version are
specified by the *`PACKAGE_NAME`* and *`PACKAGE_VERSION`* variables in
the package's *dkms.conf* file.

Due to **DKMS** [bug #538](https://github.com/dell/dkms/issues/538) and
other bugs, the recommended way to add **Comedi** to **DKMS** is to
unpack the (unofficial) release tarball manually into the */usr/src*
directory then add it to **DKMS** using a *package-name/package-version*
speficifier, using the following commands:

```
sudo tar -C /usr/src -xaf comedi-${COMEDI_VERSION}.tar.gz
sudo dkms add comedi/${COMEDI_VERSION}
```

For example:

```
sudo tar -C /usr/src -xaf comedi-0.7.76.1.395-250b.tar.gz
sudo dkms add comedi/0.7.76.1.395-250b
```

The **dkms add** command also allows the package to be specified by
supplying the actual tarball, which it will unpack and copy to
*/usr/src/*, but due to bug \#538, the *.tarball-version* file will be
omitted from the copy.  **Comedi**'s **configure** script uses that file
to set the version string used by the main **comedi** module.  Also,
some versions of **DKMS** before version 3.0.11 think the tarball is
invalid due to another bug.

Once **Comedi** has been added, The following command may be used to
automatically build and install the modules (unless automatic
installation has been disabled by the presence of a file
*/etc/dkms/no-autoinstall*):

```
sudo dkms autoinstall
```

> [!NOTE]
> If building on **Ubuntu** for the first time when **Secure Boot** is
> enabled, and it generates a new signing key (MOK) and certificate,
> **dkms** will open a dialog to enroll the new MOK.  The dialog will
> ask for a password to be used when enrolling the MOK after a reboot.
> See section *[Enrolling The DKMS Signing Certificate][dkmsenroll].*

More control over adding, removing, building, unbuilding (since **dkms**
version 2.8.2), installing, and uninstalling, for multiple kernel
versions or individual kernel versions is possible using the various
**dkms** subcommands.  See the **dkms** man page for details.

Use the following command to check the status of each package managed by
**DKMS** on each kernel:

```
sudo dkms status
```

A status of **`installed`** at the end of a line means that the
indicated package name and version has been installed for the indicated
kernel version and architecture.

#### Enrolling The DKMS Signing Certificate

If using **Secure Boot**, the Machine Owner Key (MOK) used by **DKMS**
to sign the modules it builds needs to be enrolled into the UEFI BIOS.
This can be done using the **mokutil** program.  (Use the distribution's
package manager to install **mokutil**, if it is missing.)

First, determine the pathname of the certificate used by **DKMS**.  By
default, this will be _/var/lib/dkms/mok.pub_ except on **Ubuntu**
installations, where it defaults to */var/lib/shim-signed/mok/MOK.der*.
(Also, for **Gentoo** systems the default is set by the
*`MODULES_SIGN_CERT`* variable in _/etc/portage/make.conf_.)  The
default may be overridden by setting the *`mok_certificate`* variable in
the _/etc/dkms/framework.conf_ or the
_/etc/dkms/framework.conf.d/\*.conf_ files.

The following command may be used to check if the certificate has
already been enrolled:

```
sudo mokutil --test-key /path/to/dkms/certificate
```

where **`/path/to/dkms/certificate`** depends on the location of the
certificate, for example:

```
sudo mokutil --test-key /var/lib/dkms/mok.pub
```

> [!NOTE]
> On **Ubuntu** systems, if **dkms** generates a new key and
> certificate, it will initiate the enrollment automatically.

If the certificate needs to be enrolled manually, use the following
command to do so:

```
sudo mokutil --import /path/to/dkms/certificate
```

where **`/path/to/dkms/certificate`** is the certificate to be enrolled.
In addition to **sudo** possibly prompting for the user's password,
**mokutil** will prompt to input a password to be used for MOK
enrollment, and then prompt again to confirm it was entered correctly.

> [!NOTE]
> The MOK enrollment process involves the use of a temporary password
> that needs to be repeated during the next system boot, and it may be
> using a default US English keyboard layout at that time.  If you do
> not know the positions of the characters on the US English keyboard
> layout, use something fairly universal (such as a string of ASCII
> digit characters 0 to 9) for the temporary password.

After requesting enrollment of the MOK, reboot the system to
complete the process:

* On the next boot, **MOK Management** is launched and you have to
  choose **`Enroll MOK`**.
* Choose **`Continue`** to enroll the key or **`View key 0`** to show
  the keys already enrolled.
* Confirm enrollment by selecting **`Yes`**.
* You will need to enter the MOK enrollment password that was supplied
  to **mokutil** earlier.
* The new key is enrolled, and the system asks you to reboot.

After reboot, check if the certificate has been enrolled, using
**mokutil**'s **`--test-key`** option as described earlier.




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
[dkmsenroll]: #enrolling-the-dkms-signing-certificate "Enrolling the DKMS signing certificate"
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
[dkmsmokdeb]: #setting-up-module-signing-support-on-debian "Setting up module signing support on Debian"
[systemd]: https://systemd.io/ "Systemd"
