Name: libehca
Version: 1.2.2
Release: 0.1.g69e1a88
Summary: IBM InfiniBand HCA (ehca) Userspace Driver

Group: System Environment/Libraries
License: GPL/BSD
Url: http://www.openfabrics.org/
Source: http://www.openfabrics.org/downloads/libehca-1.2.2-0.1.g69e1a88.tar.gz
BuildRoot: %(mktemp -ud %{_tmppath}/%{name}-%{version}-%{release}-XXXXXX)

BuildRequires: libibverbs-devel >= 1.1-0.1.rc2

%description
libehca provides a device-specific userspace driver for IBM HCAs
for use with the libibverbs library.

%package devel-static
Summary: Development files for the libehca driver
Group: System Environment/Libraries
Requires: %{name} = %{version}-%{release}

%description devel-static
Static version of libehca that may be linked directly to an
application, which may be useful for debugging.

%prep
%setup -q -n %{name}-1.2.2

%build
%configure
make %{?_smp_mflags}

%install
rm -rf $RPM_BUILD_ROOT
make DESTDIR=%{buildroot} install
# remove unpackaged files from the buildroot
rm -f $RPM_BUILD_ROOT%{_libdir}/*.la $RPM_BUILD_ROOT%{_libdir}/libehca.so

%clean
rm -rf $RPM_BUILD_ROOT

%files
%defattr(-,root,root,-)
%{_libdir}/libehca-rdmav2.so
%{_sysconfdir}/libibverbs.d/ehca.driver
#%doc AUTHORS COPYING ChangeLog README

%files devel-static
%defattr(-,root,root,-)
%{_libdir}/libehca.a

%changelog
* Mon Jul  9 2007 Vladimir Sokolovsky <vlad@mellanox.co.il> - 1.0-0
- Initial packaging
