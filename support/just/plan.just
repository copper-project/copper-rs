# Shared schedule visualizer used by the workspace and example-local Justfiles.
PLAN_WORKSPACE_ROOT := shell('git -C "$1" rev-parse --show-toplevel', justfile_directory())

# Render the generated schedule for a config in the invocation directory.
plan mission="" features="" config="":
  #!/usr/bin/env bash
  set -euo pipefail

  invocation_dir="{{invocation_directory()}}"
  config_value="{{config}}"
  config_value="${config_value#config=}"
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
  mission_value="{{mission}}"
  mission_value="${mission_value#mission=}"
  features_value="{{features}}"
  features_value="${features_value#features=}"
  if [[ -n "$mission_value" ]]; then
    args+=(--mission "$mission_value")
  fi
  if [[ -n "$features_value" ]]; then
    args+=(--features "$features_value")
  fi

  cd "{{PLAN_WORKSPACE_ROOT}}"
  cargo run -p cu29-runtime --bin cu29-plan -- "${args[@]}"
