#!/usr/bin/python
import re

_distributor_id_file_re = re.compile(r"(?:DISTRIB_ID\s*=)\s*(.*)", re.I)
_release_file_re = re.compile(r"(?:DISTRIB_RELEASE\s*=)\s*(.*)", re.I)
_codename_file_re = re.compile(r"(?:DISTRIB_CODENAME\s*=)\s*(.*)", re.I)

def linux_distribution():
    # check for the Debian/Ubuntu /etc/lsb-release file first, needed so
    # that the distribution doesn't get identified as Debian.
    try:
        with open("/etc/lsb-release", "r") as etclsbrel:
            for line in etclsbrel:
                m = _distributor_id_file_re.search(line)
                if m:
                    _u_distname = m.group(1).strip()
                m = _release_file_re.search(line)
                if m:
                    _u_version = m.group(1).strip()
                m = _codename_file_re.search(line)
                if m:
                    _u_id = m.group(1).strip()
            if _u_distname and _u_version:
                return (_u_distname, _u_version, _u_id)
    except (EnvironmentError, UnboundLocalError):
            print("Error getting linux distribution")
            exit()

