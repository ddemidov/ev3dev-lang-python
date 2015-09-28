#!/bin/sh

# wget https://pypi.python.org/packages/source/p/python-ev3dev/python-ev3dev-0.2.2.post1.tar.gz
if [ -z "$1" ]; then
  echo "Usage: $0 <source.tar.gz>"
  exit 1
fi

src=$1

wd=$PWD
td=$(mktemp -d)

cd "${td}"
cp "${wd}/$src" "${td}/"

sf=$(basename "$src")
sn=$(basename "$sf" ".tar.gz")
tar xzf "$sf"

# please debians idea of the world
pkg=$(echo "${sn}" | cut -d'-' -f1,2)
ver=$(echo "${sn}" | cut -d'-' -f3)
dn="${pkg}_${ver}"
mv "${sn}" "${dn}"
tar cvzf "${dn}.orig.tar.gz" "${dn}" 


cp -r "${wd}/debian" "${td}/${dn}/"
cd "${dn}"
dpkg-buildpackage -us -uc
cd ..

lintian -EvIm --pedantic --show-overrides --color=auto --profile=debian *.changes

echo "packages created in: ${td}"

cd "${wd}"
