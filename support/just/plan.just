# Shared schedule visualizer used by the workspace and example-local Justfiles.
PLAN_WORKSPACE_ROOT := source_directory() + "/../.."

# Render the generated schedule for a config in the invocation directory.
[positional-arguments]
plan *options:
  #!/usr/bin/env bash
  set -euo pipefail

  invocation_dir="{{invocation_directory()}}"
  mission_value=""
  features_value=""
  config_value=""
  log_value=""
  bin_value=""
  log_features_value=""
  for option in "$@"; do
    case "$option" in
      mission=*) mission_value="${option#mission=}" ;;
      features=*) features_value="${option#features=}" ;;
      config=*) config_value="${option#config=}" ;;
      log=*) log_value="${option#log=}" ;;
      bin=*) bin_value="${option#bin=}" ;;
      logfeatures=*) log_features_value="${option#logfeatures=}" ;;
      *)
        echo "Unknown plan option '$option'; expected mission=<id>, features=<a,b>, config=<file>, log=<file>, bin=<logreader>, or logfeatures=<a,b>." >&2
        exit 2
        ;;
    esac
  done

  if [[ -n "$config_value" ]]; then
    if [[ "$config_value" = /* ]]; then
      cfg_path="$config_value"
    else
      cfg_path="${invocation_dir}/${config_value}"
    fi
  elif [[ -f "${invocation_dir}/copperconfig.ron" ]]; then
    cfg_path="${invocation_dir}/copperconfig.ron"
  elif [[ -f "${invocation_dir}/mcu_config.ron" ]]; then
    cfg_path="${invocation_dir}/mcu_config.ron"
  else
    candidates=()
    while IFS= read -r candidate; do
      candidates+=("$candidate")
    done < <(find "$invocation_dir" -maxdepth 1 -type f -name '*config.ron' | sort)
    if [[ "${#candidates[@]}" -eq 1 ]]; then
      cfg_path="${candidates[0]}"
    else
      echo "Could not select one Copper config in ${invocation_dir}; pass config=<file>." >&2
      if [[ "${#candidates[@]}" -eq 0 ]]; then
        echo "Candidates: none" >&2
      else
        printf 'Candidate: %s\n' "${candidates[@]}" >&2
      fi
      exit 1
    fi
  fi
  if [[ ! -f "$cfg_path" ]]; then
    echo "No Copper config found at ${cfg_path}" >&2
    exit 1
  fi

  output_dir="$(dirname "$cfg_path")"
  args=("$cfg_path" --output "${output_dir}/plan.svg" --open)
  if [[ -n "$mission_value" ]]; then
    args+=(--mission "$mission_value")
  fi
  if [[ -n "$features_value" ]]; then
    args+=(--features "$features_value")
  fi

  if [[ -n "$log_value" ]]; then
    [[ -n "$bin_value" ]] || { echo "bin=<logreader> is required with log=<file>." >&2; exit 2; }
    if [[ "$log_value" = /* ]]; then
      log_path="$log_value"
    else
      log_path="${invocation_dir}/${log_value}"
    fi
    [[ "$log_path" != *_0.copper ]] || log_path="${log_path%_0.copper}.copper"
    log_slab="${log_path%.copper}_0.copper"
    [[ -f "$log_path" || -f "$log_slab" ]] || { echo "Copper log not found: $log_path" >&2; exit 1; }
    stats_path="${output_dir}/target/cu29_plan_logstats.json"
    mkdir -p "$(dirname "$stats_path")"
    logreader_args=(run --bin "$bin_value")
    [[ -z "$log_features_value" ]] || logreader_args+=(--features "$log_features_value")
    logreader_args+=(-- "$log_path" log-stats --config "$cfg_path" --output "$stats_path")
    [[ -z "$mission_value" ]] || logreader_args+=(--mission "$mission_value")
    config_features_value="$features_value"
    [[ -n "$config_features_value" ]] || config_features_value="$log_features_value"
    [[ -z "$config_features_value" ]] || logreader_args+=(--features "$config_features_value")
    (cd "$invocation_dir" && cargo "${logreader_args[@]}")
    args+=(--logstats "$stats_path")
  fi

  cd "{{PLAN_WORKSPACE_ROOT}}"
  cargo run -p cu29-runtime --bin cu29-plan -- "${args[@]}"

# Render observed timing from this example's default local Copper log.
[positional-arguments]
plan-log *options:
  #!/usr/bin/env bash
  set -euo pipefail

  invocation_dir="{{invocation_directory()}}"
  defaults="$(python3 "{{PLAN_WORKSPACE_ROOT}}/support/ci/plan_defaults.py" "$invocation_dir")"
  bin_default="$(printf '%s\n' "$defaults" | sed -n '1p')"
  features_default="$(printf '%s\n' "$defaults" | sed -n '2p')"
  log_default="$(printf '%s\n' "$defaults" | sed -n '3p')"
  if [[ -z "$bin_default" || -z "$log_default" ]]; then
    echo "Could not discover plan-log defaults for ${invocation_dir}." >&2
    exit 1
  fi

  args=("log=${log_default}" "bin=${bin_default}")
  if [[ -n "$features_default" ]]; then
    args+=("features=${features_default}" "logfeatures=${features_default}")
  fi
  args+=("$@")
  (cd "$invocation_dir" && just plan "${args[@]}")
