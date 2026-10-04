#!/usr/bin/env bash
set -euo pipefail
hardware_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_options=()
for proxy_variable in HTTP_PROXY HTTPS_PROXY ALL_PROXY http_proxy https_proxy all_proxy; do
  if [[ -n "${!proxy_variable:-}" ]]; then
    build_options+=(--build-arg "$proxy_variable")
  fi
done
build_network="${DOCKER_BUILD_NETWORK:-}"
if [[ -z "$build_network" && "$(uname -s)" == "Linux" ]]; then
  proxy_endpoints="${HTTP_PROXY:-}${HTTPS_PROXY:-}${ALL_PROXY:-}${http_proxy:-}${https_proxy:-}${all_proxy:-}"
  if [[ "$proxy_endpoints" == *127.0.0.1* || "$proxy_endpoints" == *localhost* ]]; then
    build_network=host
  fi
fi
if [[ -n "$build_network" ]]; then build_options+=(--network "$build_network"); fi
exec docker build -t "${IMAGE:-duojin01:hardware-humble}" "${build_options[@]}" "$@" "$hardware_root"
