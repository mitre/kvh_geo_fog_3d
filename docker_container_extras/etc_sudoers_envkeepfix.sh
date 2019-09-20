#!/bin/bash

# Will add in env_keep right below env_reset, and verify correct sudo formatting using visudo

if [ ${EUID} != 0 ]; then
    echo "Process must be run as root!"
    exit 1
fi

SUDOER_TMP=$(mktemp)
cat /etc/sudoers > ${SUDOER_TMP}
sed -i -e 's/Defaults\s*env_reset/Defaults\tenv_reset\nDefaults\tenv_keep="http_proxy HTTP_PROXY https_proxy HTTPS_PROXY ftp_proxy FTP_PROXY no_proxy NO_PROXY"/' ${SUDOER_TMP}
visudo -c -f ${SUDOER_TMP} && cat ${SUDOER_TMP} > /etc/sudoers
rm ${SUDOER_TMP}
