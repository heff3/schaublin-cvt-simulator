FROM debian:bookworm-slim

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install --yes --no-install-recommends \
        build-essential \
        linuxcnc-uspace-dev \
    && rm -rf /var/lib/apt/lists/*

RUN if ! command -v halcompile >/dev/null 2>&1; then \
        echo "halcompile not found on PATH" >&2; \
        exit 1; \
    fi

WORKDIR /workspace