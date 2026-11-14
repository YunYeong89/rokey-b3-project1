# syntax=docker/dockerfile:1
FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential git python3-colcon-common-extensions python3-rosdep \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Resolve workspace manifests without storing robot meshes or source in the image.
RUN --mount=type=bind,source=ws_cobot_pjt/src,target=/tmp/cobot-src \
    source /opt/ros/jazzy/setup.bash \
    && if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update --rosdistro jazzy \
    && apt-get update \
    && rosdep install --from-paths /tmp/cobot-src --ignore-src --rosdistro jazzy -y \
    && rosdep check --from-paths /tmp/cobot-src --ignore-src --rosdistro jazzy \
    && rm -rf /var/lib/apt/lists/*

# The gripper's documented Python dependency is missing from its package.xml.
RUN python3 -m venv --system-site-packages /opt/cobot-venv \
    && /opt/cobot-venv/bin/pip install --no-cache-dir pymodbus==3.6.9
ENV PYTHONPATH=/opt/cobot-venv/lib/python3.12/site-packages

COPY --chmod=755 docker-entrypoint.sh /cobot-entrypoint.sh
RUN printf '\nsource /opt/ros/jazzy/setup.bash\n[ ! -f /workspace/install/setup.bash ] || source /workspace/install/setup.bash\n' >> /root/.bashrc

WORKDIR /workspace
ENTRYPOINT ["/cobot-entrypoint.sh"]
CMD ["sleep", "infinity"]
