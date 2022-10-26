#!/usr/bin/bash
IFS=', ' read -r -a debs <<<$(echo *.deb | tr ' ' ',')
for deb in "${debs[@]}"; do
    curl -u "sunny-ctirobot:admin" \
        -H "Content-Type: multipart/form-data " --data-binary "@./${deb}" \
        "http://192.168.100.48:8081/repository/apt/"
done

IFS=', ' read -r -a ddebs <<<$(echo *.ddeb | tr ' ' ',')
for ddeb in "${ddebs[@]}"; do
    curl -u "sunny-ctirobot:admin" \
        -H "Content-Type: multipart/form-data " --data-binary "@./${ddeb}" \
        "http://192.168.100.48:8081/repository/apt/"
done
