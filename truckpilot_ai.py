"""
TruckPilot AI Helper — alle Provider via Helicone Gateway.
Keys aus .env laden:  from dotenv import load_dotenv; load_dotenv()

Nutzung:
    from truckpilot_ai import chat, HeliconeGateway
    reply = chat("kimi-k2.6", "Hallo Welt")
"""

import os
from openai import OpenAI

try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:
    pass

HELICONE_URL = os.getenv("HELICONE_BASE_URL", "http://100.103.91.121:8585")
HELICONE_KEY = os.getenv("HELICONE_API_KEY")

PROVIDERS = {
    "openai": {
        "key": os.getenv("OPENAI_API_KEY"),
        "base": "https://api.openai.com",
        "default_model": "gpt-4o-mini",
    },
    "deepseek": {
        "key": os.getenv("DEEPSEEK_API_KEY"),
        "base": "https://api.deepseek.com",
        "default_model": "deepseek-chat",
    },
    "moonshot": {
        "key": os.getenv("MOONSHOT_API_KEY"),
        "base": "https://api.moonshot.ai",
        "default_model": "kimi-k2.6",
    },
    "openrouter": {
        "key": os.getenv("OPENROUTER_API_KEY"),
        "base": "https://openrouter.ai/api",
        "default_model": "openai/gpt-4o-mini",
    },
}


def chat(model: str, message: str, provider: str | None = None, **kwargs):
    """Ein Request via Helicone Gateway. Provider wird aus Model-Name erraten."""
    if provider is None:
        for pname, pinfo in PROVIDERS.items():
            if model.startswith(pname + "/") or any(
                model.startswith(prefix) for prefix in _model_prefixes(pname)
            ):
                provider = pname
                break
        else:
            provider = "openai"  # fallback

    p = PROVIDERS[provider]
    client = OpenAI(
        base_url=f"{HELICONE_URL}/v1/gateway/oai/v1",
        api_key=p["key"],
        default_headers={
            "Helicone-OpenAI-Api-Base": p["base"],
            "helicone-auth": f"Bearer {HELICONE_KEY}",
        },
    )
    resp = client.chat.completions.create(
        model=model,
        messages=[{"role": "user", "content": message}],
        **kwargs,
    )
    return resp.choices[0].message.content


def _model_prefixes(provider: str) -> list[str]:
    prefixes = {
        "openai": ["gpt-", "o1", "o3", "o4"],
        "deepseek": ["deepseek-"],
        "moonshot": ["kimi-", "moonshot-"],
    }
    return prefixes.get(provider, [])


# ─── Schnelltest ───────────────────────────────────────────
if __name__ == "__main__":
    tests = [
        ("gpt-4o-mini", "Say hi in one word"),
        ("deepseek-chat", "Say hi in one word"),
        ("kimi-k2.6", "Say hi in one word"),
    ]
    for model, msg in tests:
        try:
            result = chat(model, msg)
            print(f"✅ {model}: {result}")
        except Exception as e:
            print(f"❌ {model}: {e}")
