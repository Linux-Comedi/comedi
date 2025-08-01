
#   COMEDI
### The Linux Control and Measurement Device Interface
##  David Schleef <ds@schleef.org>
---

## About Comedi

**[Comedi][]** is a collection of drivers for data acquisition hardware.
These drivers work with **[Linux][]**, and also with **Linux** combined
with the real-time extensions **[RTAI][]** and **RTLinux** (dead
project, not to be confused with **Real-Time Linux** (**PREEMPT_RT**)).
The **Comedi** core, which ties all the driver together, allows
applications to be written that are completely hardware-independent.

**Comedi** supports a variety of data acquisition hardware; an
incomplete list may be found at
<http://www.comedi.org/hardware.html#hw-comedi-org>.

This distribution contains just the **Comedi** kernel modules.  You will
almost certainly also want to download **Comedilib**, which is a user
space library, a few utilities, and some example programs.

**Comedi** may be freely distributed and modified in accordance with the
[GNU General Public License v2.0][gplv2] or later.

The person behind all this misspelled humor is David Schleef
<ds@schleef.org>.

## Installation

For installation instructions, look at the file *[INSTALL.md][]*.

## Running

For instructions on running **Comedi**, see the section *[Running
Comedi][runtime]* in *[INSTALL.md][]*.

# Mailing List

Questions about **Comedi** should be sent to the Comedi mailing list,
<comedi_list@googlegroups.com>.  It is necessary to join the group
before posting (see below).  It is also possible to post to the list
using the web interface (see below).  Mailing the maintainer directly is
always acceptable, but since the mailing list is archived and questions
are often answered more quickly by others, the mailing list is
preferred.

To subscribe to and unsubscribe from the mailing list, or to read or
post messages via the web interface, go to
<http://groups.google.com/group/comedi_list>.  Alternatively, you
can send a blank email to <comedi_list+subscribe@googlegroups.com>
to subscribe, or to <comedi_list+unsubscribe@googlegroups.com> to
unsubscribe (making sure the "From:" email address matches the
address you originally subscribed with!).

Traffic on the list is light, and mainly questions/answers about
comedi installation, bugs, and programming.  General questions
about data acquisition are also welcome.

## More Information

The web site for the **Comedi** project is at <https://www.comedi.org/>.
This contains a download section for official releases for **Comedi**
and related projects, although the official releases for **Comedi**
itself are obsolete.  It also includes links to the **Git** repositories
where **Comedi** and its related projects are maintained.

Often bugfixes and new features that are not in the current release can
be found in the **Git** repository.  **Git** snapshots can be created
automatically at
<https://github.com/Linux-Comedi/comedi/tarball/master>.  The **Git**
repository can be accessed anonymously using:

    git clone https://github.com/Linux-Comedi/comedi.git

The **Git** repository was previously hosted at [comedi.org][].  A
previously cloned repository may need its URL updating to the current
repository on [github.com][] as follows:

    cd /path/to/comedi
    git remote set-url origin https://github.com/Linux-Comedi/comedi.git

### Official Releases (Or Lack Thereof)

There have not been any official releases since **Comedi 0.7.76** on
2008-01-28, which will not build for any recent **Linux** kernels.
However, **Comedi** has been maintained in the **Git** repository that
includes support for recent **Linux** kernels (up to at least kernel
version 6.16 at the time of writing).

It is possible to generate unofficial releases from the **Git** sources
for use with **[Akmods][]** or **[DKMS][]**.  See the section
*[Generating Unofficial Releases][genrel]* in *[INSTALL.md][]* for
details.

### Related Projects

#### In-Tree Linux Kernel Comedi

There has been a version of the **Comedi** kernel modules included in
the **Linux** kernel sources since kernel version 2.6.29.  Some
distributions build these as part of their kernel binary packages, and
some distributions do not.  The internal API of the in-tree **Comedi**
modules has diverged from that of the out-of-tree ([comedi.org][])
**Comedi** modules, so the two sets of modules are incompatible with
each other and cannot be loaded at the same time.

The kernel-user space API is the same for both the in-tree version and
the [comedi.org][] version, although there are differences in the
low-level hardware drivers.

#### Comedilib

**Comedilib** is the official user-space library interface for handling
**Comedi** devices in application programs.  It includes various
utilities for configuring legacy (e.g. ISA card) **Comedi** devices
(**comedi_config**), getting information about a **Comedi** device
(**comedi_board_info**), and testing a **Comedi** device
(**comedi_test**).  The project source includes various demonstration
programs, and bindings for various programming languages.

#### comedi_calibrate

**comedi_calibrate** is a calibration utility that supports a few
**Comedi** hardware devices.

#### comedi-nonfree-firmware

A few **Comedi** devices require non-free firmware to be loaded at
run-time.  The **comedi-nonfree-firmware** package includes copies of
these firmware files.




[akmods]: https://rpmfusion.org/Packaging/KernelModules/Akmods "Akmods"
[comedi.org]: https://www.comedi.org/ "Comedi"
[install.md]: INSTALL.md "Comedi installation"
[dkms]: https://github.com/dell/dkms "Dynamic Kernel Module System (DKMS)"
[genrel]: INSTALL.md#generating-unofficial-releases "Generating unofficial releases"
[github.com]: https://github.com/ "GitHub"
[gplv2]: https://www.gnu.org/licenses/old-licenses/gpl-2.0.html "GNU GPLv2"
[linux]: https://www.kernel.org/ "Linux kernel"
[rtai]: https://www.rtai.org] "RTAI - Real Time Application Interface"
[runtime]: INSTALL.md#running-comedi "Running Comedi"
