# Native ARM64 Linux, explicit x86-64 emulation; no host binfmt changes.
ARG DEBIAN_BASE
FROM ${DEBIAN_BASE} AS build
ARG DEBIAN_SNAPSHOT
RUN rm -f /etc/apt/sources.list.d/debian.sources && printf 'deb [check-valid-until=no] http://snapshot.debian.org/archive/debian/%s/ bookworm main\ndeb [check-valid-until=no] http://snapshot.debian.org/archive/debian-security/%s/ bookworm-security main\n' "$DEBIAN_SNAPSHOT" "$DEBIAN_SNAPSHOT" > /etc/apt/sources.list && dpkg --add-architecture amd64 && apt-get update && apt-get install -y --no-install-recommends ca-certificates make cmake g++-x86-64-linux-gnu libssl-dev:amd64 libboost-dev nlohmann-json3-dev && rm -rf /var/lib/apt/lists/*
COPY host /source
RUN cmake -S /source -B /build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_SYSTEM_NAME=Linux -DCMAKE_SYSTEM_PROCESSOR=x86_64 -DCMAKE_C_COMPILER=x86_64-linux-gnu-gcc -DCMAKE_CXX_COMPILER=x86_64-linux-gnu-g++ -DOPENSSL_CRYPTO_LIBRARY=/usr/lib/x86_64-linux-gnu/libcrypto.so -DOPENSSL_INCLUDE_DIR=/usr/include && cmake --build /build --target tyr_bambu_cloud_host -j2
FROM ${DEBIAN_BASE}
ARG DEBIAN_SNAPSHOT
RUN rm -f /etc/apt/sources.list.d/debian.sources && printf 'deb [check-valid-until=no] http://snapshot.debian.org/archive/debian/%s/ bookworm main\ndeb [check-valid-until=no] http://snapshot.debian.org/archive/debian-security/%s/ bookworm-security main\n' "$DEBIAN_SNAPSHOT" "$DEBIAN_SNAPSHOT" > /etc/apt/sources.list && dpkg --add-architecture amd64 && apt-get update && apt-get install -y --no-install-recommends ca-certificates qemu-user-static libc6:amd64 libstdc++6:amd64 libgcc-s1:amd64 zlib1g:amd64 libssl3:amd64 && find /usr/bin -maxdepth 1 -name 'qemu-*-static' ! -name 'qemu-x86_64-static' -delete && rm -rf /var/lib/apt/lists/*
COPY --from=build /build/tyr_bambu_cloud_host /runtime/tyr_bambu_cloud_host
ENV PJARCZAK_BAMBU_PLUGIN_DIR=/plugin PJARCZAK_BAMBU_NETWORK_SO=/plugin/libbambu_networking.so PJARCZAK_BAMBU_SOURCE_SO=/plugin/libBambuSource.so PJARCZAK_EXPECTED_BAMBU_NETWORK_VERSION=02.05.02.58 HOME=/session SSL_CERT_FILE=/etc/ssl/certs/ca-certificates.crt LD_LIBRARY_PATH=/plugin
ENTRYPOINT ["/usr/bin/qemu-x86_64-static"]
CMD ["/runtime/tyr_bambu_cloud_host"]
