FROM ghcr.io/nap-it/vanetza-nap:latest

RUN apt-get update \
    && apt-get install -y --no-install-recommends python3 python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY obu/requirements.txt /app/requirements.txt
RUN pip3 install --no-cache-dir -r /app/requirements.txt

COPY obu/app /app
COPY obu/entrypoint.sh /entrypoint-obu.sh
COPY obu/config/vanetza-obu.ini /config.ini

RUN chmod +x /entrypoint-obu.sh

ENTRYPOINT ["/entrypoint-obu.sh"]
