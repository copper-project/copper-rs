import importlib.util
import json
import struct
import sys
import traceback


class AttrDict(dict):
    def __getattr__(self, name):
        try:
            return self[name]
        except KeyError as exc:
            raise AttributeError(name) from exc

    def __setattr__(self, name, value):
        self[name] = value

    def __delattr__(self, name):
        try:
            del self[name]
        except KeyError as exc:
            raise AttributeError(name) from exc


def _wrap(value):
    if isinstance(value, dict):
        return AttrDict({key: _wrap(item) for key, item in value.items()})
    if isinstance(value, list):
        return [_wrap(item) for item in value]
    return value


def _unwrap(value):
    if isinstance(value, dict):
        return {key: _unwrap(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_unwrap(item) for item in value]
    return value


def load_process_function(script_path):
    spec = importlib.util.spec_from_file_location("cu_python_task_user", script_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load Python task script from {script_path!r}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    process_fn = getattr(module, "process", None)
    if process_fn is None or not callable(process_fn):
        raise RuntimeError(
            f"Python task script {script_path!r} must define a callable process(input, state, output)"
        )
    return process_fn


def call_process(process_fn, request):
    input_value = _wrap(request["input"])
    state_value = _wrap(request["state"])
    output_value = _wrap(request["output"])
    process_fn(input_value, state_value, output_value)
    return {
        "state": _unwrap(state_value),
        "output": _unwrap(output_value),
    }


def call_process_json(process_fn, request_json):
    request = json.loads(request_json)
    response = call_process(process_fn, request)
    return json.dumps(response, separators=(",", ":"))


def _read_frame(reader):
    header = reader.read(4)
    if not header:
        return None
    if len(header) != 4:
        raise EOFError("Incomplete frame header")

    (length,) = struct.unpack("<I", header)
    payload = reader.read(length)
    if len(payload) != length:
        raise EOFError("Incomplete frame payload")
    return payload


def _write_frame(writer, payload):
    writer.write(struct.pack("<I", len(payload)))
    writer.write(payload)
    writer.flush()


def _write_json(writer, value):
    _write_frame(writer, json.dumps(value, separators=(",", ":")).encode("utf-8"))


def main():
    if len(sys.argv) != 2:
        raise RuntimeError("Expected script path argument")

    script_path = sys.argv[1]
    reader = sys.stdin.buffer
    writer = sys.stdout.buffer

    try:
        process_fn = load_process_function(script_path)
        _write_json(writer, {"kind": "ready"})
    except Exception:
        _write_json(writer, {"kind": "error", "message": traceback.format_exc()})
        return 1

    while True:
        try:
            frame = _read_frame(reader)
            if frame is None:
                return 0

            request = json.loads(frame.decode("utf-8"))
            kind = request.get("kind")

            if kind == "shutdown":
                return 0
            if kind != "process":
                raise RuntimeError(f"Unsupported request kind: {kind!r}")

            response = call_process(process_fn, request)
            _write_json(writer, {"kind": "result", **response})
        except Exception:
            _write_json(writer, {"kind": "error", "message": traceback.format_exc()})


if __name__ == "__main__":
    raise SystemExit(main())
