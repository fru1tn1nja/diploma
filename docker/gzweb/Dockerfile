FROM osrf/ros:humble-desktop-full AS base

# ── system deps ───────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    git curl build-essential cmake python3-dev python3-colcon-common-extensions \
    nodejs npm \
    gazebo libgazebo-dev ros-humble-gazebo-ros-pkgs\
    libjansson-dev pkg-config imagemagick\
    && rm -rf /var/lib/apt/lists/*

WORKDIR /gzweb_ws

# ── download gzweb ─────────────────────────────────────────────────────────
RUN git clone https://github.com/osrf/gzweb.git src/gzweb

# ── build gzweb ────────────────────────────────────────────────────────────
RUN cd src/gzweb \
    && ./deploy.sh -m


# ── expose static build ───────────────────────────────────────────────────
EXPOSE 8080
CMD ["node", "src/gzweb/http-server.js", "8080"]
