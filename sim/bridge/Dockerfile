FROM ghcr.io/eclipse-sumo/sumo:latest

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        gnupg \
        python3 \
        python3-pip \
    && curl -fsSL https://downloads.apache.org/arrow/KEYS \
        | gpg --dearmor -o /usr/share/keyrings/apache-arrow-apt-source.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/apache-arrow-apt-source.gpg] https://apache.jfrog.io/artifactory/arrow/ubuntu jammy main" \
        > /etc/apt/sources.list.d/apache-arrow.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        libarrow2300 \
        libparquet2300 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY sim/bridge/requirements.txt /app/requirements.txt
RUN pip3 install --no-cache-dir -r /app/requirements.txt

COPY sim/bridge/traci_bridge.py /app/traci_bridge.py

ENV SUMO_HOME=/usr/share/sumo

ENTRYPOINT ["python3", "/app/traci_bridge.py"]
