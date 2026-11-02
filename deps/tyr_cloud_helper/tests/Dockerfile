FROM debian:bookworm
RUN apt-get update && apt-get install -y --no-install-recommends cmake make g++ libssl-dev libboost-dev nlohmann-json3-dev python3 && rm -rf /var/lib/apt/lists/*
WORKDIR /source
COPY . .
RUN cmake -S . -B /build -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON && cmake --build /build -j4
CMD ["ctest", "--test-dir", "/build", "--output-on-failure"]
