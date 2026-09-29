"""Validate messages against the official VDA5050 v2.1.0 JSON schemas in /schemas."""
from __future__ import annotations

import json
from functools import lru_cache
from pathlib import Path

from jsonschema import Draft202012Validator

SCHEMA_DIR = Path(__file__).resolve().parent.parent / "schemas" / "vda5050_v2.1.0"


@lru_cache(maxsize=None)
def _validator(topic_name: str) -> Draft202012Validator:
    schema = json.loads((SCHEMA_DIR / f"{topic_name}.schema.json").read_text())
    return Draft202012Validator(schema)


def validation_errors(topic_name: str, message: dict) -> list[str]:
    """Return human-readable problems; an empty list means the message is valid."""
    return [
        f"{'/'.join(map(str, e.path)) or '<root>'}: {e.message}"
        for e in _validator(topic_name).iter_errors(message)
    ]
