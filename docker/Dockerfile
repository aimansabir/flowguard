# FlowGuard demo: the Rust core, SUMO and the live dashboard, built and run in
# Linux, so the Windows host never compiles or runs native FlowGuard code.
#
#   docker build -t flowguard -f docker/Dockerfile .
#   docker run --rm -it --init --name flowguard -p 127.0.0.1:8765:8765 flowguard
#   then open http://127.0.0.1:8765/
#
# Inside the container the server listens on 0.0.0.0; `-p 127.0.0.1:8765:8765`
# publishes it on the host's loopback only, so nothing outside the machine can
# reach it.
FROM rust:slim

# runtime libraries the SUMO wheel links against
RUN for i in 1 2 3; do apt-get update \
      && apt-get install -y --no-install-recommends libx11-6 libxext6 libxrender1 libgl1 libexpat1 ca-certificates \
      && break; sleep 10; done \
    && rm -rf /var/lib/apt/lists/*
COPY --from=ghcr.io/astral-sh/uv:0.13.5 /uv /uvx /usr/local/bin/

ENV UV_PROJECT_ENVIRONMENT=/opt/venv UV_PYTHON_INSTALL_DIR=/opt/python UV_LINK_MODE=copy \
    CARGO_TARGET_DIR=/opt/target PATH=/opt/venv/bin:$PATH PYTHONUNBUFFERED=1
WORKDIR /app
COPY . .
RUN uv sync --locked

EXPOSE 8765
ENTRYPOINT ["python", "-m", "flowguard_sim.live", "--host", "0.0.0.0"]
CMD ["--controller", "maxpressure_hyst_cap", "--guard", "v1", "--faults", "smoke", "--seconds", "300"]
