FROM eur0c.laas.fr:5000/gepetto/buildfarm/robotpkg:16.04

RUN apt-get update -qqy \
 && apt-get install -qqy \
    robotpkg-py27-pinocchio \
 && rm -rf /var/lib/apt/lists/*
