def elog(role: str, msg: str) -> str:
    tag = {
        'ROOT': '🌳',
        'NODE': '🧩',
        'AGENT': '🤖',
        'CTRL': '🎛️',
        'DATA': '📦',
        'WARN': '⚠️',
        'ERR' : '🛑'
    }.get(role.upper(), '🧩')
    return f"[{tag} {role.upper()}] {msg}"

