import importlib.util
import struct
import sys
import traceback
from dataclasses import dataclass


class AttrDict(dict):
    def __init__(self, *args, on_mutate=None, **kwargs):
        object.__setattr__(self, "_cu_on_mutate", on_mutate)
        super().__init__()
        if args or kwargs:
            items = dict(*args, **kwargs)
            for key, value in items.items():
                super().__setitem__(key, _wrap(value, on_mutate=on_mutate))

    def _cu_mark_mutated(self):
        callback = object.__getattribute__(self, "_cu_on_mutate")
        if callback is not None:
            callback()

    def __getattr__(self, name):
        try:
            return self[name]
        except KeyError as exc:
            raise AttributeError(name) from exc

    def __setattr__(self, name, value):
        if name.startswith("_cu_"):
            object.__setattr__(self, name, value)
            return
        self[name] = value

    def __delattr__(self, name):
        if name.startswith("_cu_"):
            object.__delattr__(self, name)
            return
        try:
            del self[name]
        except KeyError as exc:
            raise AttributeError(name) from exc

    def __setitem__(self, key, value):
        self._cu_mark_mutated()
        super().__setitem__(
            key, _wrap(value, on_mutate=object.__getattribute__(self, "_cu_on_mutate"))
        )

    def __delitem__(self, key):
        self._cu_mark_mutated()
        super().__delitem__(key)

    def clear(self):
        if self:
            self._cu_mark_mutated()
        super().clear()

    def pop(self, key, *args):
        if key in self:
            self._cu_mark_mutated()
        return super().pop(key, *args)

    def popitem(self):
        self._cu_mark_mutated()
        return super().popitem()

    def setdefault(self, key, default=None):
        if key in self:
            return super().setdefault(key)
        self._cu_mark_mutated()
        return super().setdefault(
            key, _wrap(default, on_mutate=object.__getattribute__(self, "_cu_on_mutate"))
        )

    def update(self, *args, **kwargs):
        items = dict(*args, **kwargs)
        for key, value in items.items():
            self[key] = value


class AttrList(list):
    def __init__(self, iterable=(), on_mutate=None):
        object.__setattr__(self, "_cu_on_mutate", on_mutate)
        super().__init__(_wrap(item, on_mutate=on_mutate) for item in iterable)

    def _cu_mark_mutated(self):
        callback = object.__getattribute__(self, "_cu_on_mutate")
        if callback is not None:
            callback()

    def __setitem__(self, index, value):
        self._cu_mark_mutated()
        if isinstance(index, slice):
            super().__setitem__(
                index,
                [_wrap(item, on_mutate=object.__getattribute__(self, "_cu_on_mutate")) for item in value],
            )
        else:
            super().__setitem__(
                index, _wrap(value, on_mutate=object.__getattribute__(self, "_cu_on_mutate"))
            )

    def __delitem__(self, index):
        self._cu_mark_mutated()
        super().__delitem__(index)

    def append(self, value):
        self._cu_mark_mutated()
        super().append(_wrap(value, on_mutate=object.__getattribute__(self, "_cu_on_mutate")))

    def extend(self, values):
        self._cu_mark_mutated()
        super().extend(
            _wrap(value, on_mutate=object.__getattribute__(self, "_cu_on_mutate"))
            for value in values
        )

    def insert(self, index, value):
        self._cu_mark_mutated()
        super().insert(index, _wrap(value, on_mutate=object.__getattribute__(self, "_cu_on_mutate")))

    def pop(self, index=-1):
        self._cu_mark_mutated()
        return super().pop(index)

    def remove(self, value):
        self._cu_mark_mutated()
        super().remove(value)

    def clear(self):
        if self:
            self._cu_mark_mutated()
        super().clear()


class MessageDict(AttrDict):
    def __init__(self, mapping):
        payload_present = mapping.get("__cu_payload_present__", mapping.get("payload") is not None)
        payload_template = mapping.get("__cu_payload_template__")
        clean = {
            key: value
            for key, value in mapping.items()
            if key not in {"__cu_payload_present__", "__cu_payload_template__"}
        }
        super().__init__()
        object.__setattr__(self, "_cu_payload_present", payload_present)
        object.__setattr__(self, "_cu_payload_touched", False)
        object.__setattr__(self, "_cu_payload_template", payload_template)
        for key, value in clean.items():
            if key == "payload" and value is None:
                dict.__setitem__(self, key, None)
            else:
                callback = self._cu_mark_payload_touched if key == "payload" else None
                dict.__setitem__(self, key, _wrap(value, on_mutate=callback))

    def _cu_mark_payload_touched(self):
        object.__setattr__(self, "_cu_payload_touched", True)

    def _cu_materialize_payload(self):
        payload = dict.get(self, "payload")
        if payload is None:
            template = object.__getattribute__(self, "_cu_payload_template")
            if template is None:
                return None
            payload = _wrap(template, on_mutate=self._cu_mark_payload_touched)
            dict.__setitem__(self, "payload", payload)
        return payload

    def __getattr__(self, name):
        if name == "payload":
            return self._cu_materialize_payload()
        return super().__getattr__(name)

    def __getitem__(self, key):
        if key == "payload":
            return self._cu_materialize_payload()
        return super().__getitem__(key)

    def get(self, key, default=None):
        if key == "payload":
            payload = self._cu_materialize_payload()
            return default if payload is None else payload
        return super().get(key, default)

    def __setattr__(self, name, value):
        if name.startswith("_cu_"):
            object.__setattr__(self, name, value)
            return
        if name == "payload":
            self._cu_mark_payload_touched()
            dict.__setitem__(self, "payload", _wrap(value, on_mutate=self._cu_mark_payload_touched))
            return
        super().__setattr__(name, value)

    def __setitem__(self, key, value):
        if key == "payload":
            self._cu_mark_payload_touched()
            dict.__setitem__(self, "payload", _wrap(value, on_mutate=self._cu_mark_payload_touched))
            return
        super().__setitem__(key, value)


def _wrap(value, on_mutate=None, output_mode=False):
    if isinstance(value, (MessageDict, AttrDict, AttrList)):
        return value
    if isinstance(value, dict):
        if output_mode and (
            "__cu_payload_present__" in value or "__cu_payload_template__" in value
        ):
            return MessageDict(value)
        return AttrDict(
            {
                key: _wrap(item, on_mutate=on_mutate, output_mode=output_mode)
                for key, item in value.items()
            },
            on_mutate=on_mutate,
        )
    if isinstance(value, list):
        return AttrList(
            [_wrap(item, on_mutate=on_mutate, output_mode=output_mode) for item in value],
            on_mutate=on_mutate,
        )
    return value


def _unwrap(value):
    if isinstance(value, MessageDict):
        result = {key: _unwrap(item) for key, item in value.items() if key != "payload"}
        if object.__getattribute__(value, "_cu_payload_present") or object.__getattribute__(
            value, "_cu_payload_touched"
        ):
            result["payload"] = _unwrap(dict.get(value, "payload"))
        else:
            result["payload"] = None
        return result
    if isinstance(value, AttrDict):
        return {key: _unwrap(item) for key, item in value.items()}
    if isinstance(value, AttrList):
        return [_unwrap(item) for item in value]
    if isinstance(value, dict):
        return {key: _unwrap(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_unwrap(item) for item in value]
    return value


@dataclass(slots=True, frozen=True)
class ProcessRequest:
    input: object
    state: object
    output: object


@dataclass(slots=True, frozen=True)
class ShutdownRequest:
    pass


@dataclass(slots=True, frozen=True)
class ReadyResponse:
    cbor2_accelerated: bool


@dataclass(slots=True, frozen=True)
class ProcessResultResponse:
    state: object
    output: object

    @classmethod
    def from_process_result(cls, value):
        if not isinstance(value, dict):
            raise TypeError(f"Unsupported process result type: {type(value).__name__}")
        try:
            return cls(state=value["state"], output=value["output"])
        except KeyError as exc:
            raise RuntimeError(f"Process result is missing field {exc.args[0]!r}") from exc


@dataclass(slots=True, frozen=True)
class ErrorResponse:
    message: str


def _decode_request(value):
    if not isinstance(value, dict):
        raise RuntimeError(f"Expected request mapping, got {type(value).__name__}")

    kind = value.get("kind")
    if kind == "process":
        return _coerce_process_request(value)
    if kind == "shutdown":
        return ShutdownRequest()

    raise RuntimeError(f"Unsupported request kind: {kind!r}")


def _encode_response(value):
    if isinstance(value, ReadyResponse):
        return {"kind": "ready", "cbor2_accelerated": value.cbor2_accelerated}
    if isinstance(value, ProcessResultResponse):
        return {"kind": "result", "state": value.state, "output": value.output}
    if isinstance(value, ErrorResponse):
        return {"kind": "error", "message": value.message}
    raise TypeError(f"Unsupported response type: {type(value).__name__}")


def _coerce_process_request(value):
    if isinstance(value, ProcessRequest):
        return value
    if isinstance(value, dict):
        try:
            return ProcessRequest(
                input=value["input"],
                state=value["state"],
                output=value["output"],
            )
        except KeyError as exc:
            raise RuntimeError(f"Process request is missing field {exc.args[0]!r}") from exc
    raise TypeError(f"Unsupported process request type: {type(value).__name__}")


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
    request = _coerce_process_request(request)
    input_value = _wrap(request.input)
    state_value = _wrap(request.state)
    output_value = _wrap(request.output, output_mode=True)
    process_fn(input_value, state_value, output_value)
    return {
        "state": _unwrap(state_value),
        "output": _unwrap(output_value),
    }


def _load_cbor2():
    try:
        import cbor2
    except ImportError as exc:
        raise RuntimeError(
            "cu_python_task process mode requires the Python package `cbor2`"
        ) from exc

    accelerated = (
        getattr(cbor2.loads, "__module__", "") == "_cbor2"
        and getattr(cbor2.dumps, "__module__", "") == "_cbor2"
    )
    return cbor2, accelerated


def _read_exact(reader, length):
    data = reader.read(length)
    if len(data) != length:
        raise EOFError("Incomplete frame payload")
    return data


def _read_frame(reader):
    header = reader.read(4)
    if not header:
        return None
    if len(header) != 4:
        raise EOFError("Incomplete frame header")

    (length,) = struct.unpack("<I", header)
    return _read_exact(reader, length)


def _write_frame(writer, payload):
    writer.write(struct.pack("<I", len(payload)))
    writer.write(payload)
    writer.flush()


def _write_cbor(writer, cbor2, value):
    _write_frame(writer, cbor2.dumps(_encode_response(value)))


def main():
    if len(sys.argv) != 2:
        raise RuntimeError("Expected script path argument")

    script_path = sys.argv[1]
    reader = sys.stdin.buffer
    writer = sys.stdout.buffer

    cbor2 = None
    try:
        cbor2, accelerated = _load_cbor2()
        process_fn = load_process_function(script_path)
        _write_cbor(writer, cbor2, ReadyResponse(cbor2_accelerated=accelerated))
    except Exception:
        if cbor2 is not None:
            _write_cbor(writer, cbor2, ErrorResponse(message=traceback.format_exc()))
        else:
            traceback.print_exc()
        return 1

    while True:
        try:
            frame = _read_frame(reader)
            if frame is None:
                return 0

            request = _decode_request(cbor2.loads(frame))

            if isinstance(request, ShutdownRequest):
                return 0

            response = ProcessResultResponse.from_process_result(
                call_process(process_fn, request)
            )
            _write_cbor(writer, cbor2, response)
        except Exception:
            _write_cbor(writer, cbor2, ErrorResponse(message=traceback.format_exc()))


if __name__ == "__main__":
    raise SystemExit(main())
