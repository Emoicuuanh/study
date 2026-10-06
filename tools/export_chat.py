"""Trich lich su hoi thoai Claude Code ra Markdown doc duoc.

Claude Code luu transcript dang JSONL tai:
    ~/.claude/projects/<duong-dan-du-an-doi-dau-gach>/<session-id>.jsonl

Moi dong la mot ban ghi JSON. Ngoai 'user' va 'assistant' con nhieu loai ban ghi
noi bo (attachment, last-prompt, ai-title, queue-operation...) - bo het.

Dung:
    python3 tools/export_chat.py                      # phien moi nhat
    python3 tools/export_chat.py --list               # liet ke cac phien
    python3 tools/export_chat.py --session <id>
    python3 tools/export_chat.py --full               # giu ca ket qua tool
    python3 tools/export_chat.py -o /duong/dan.md
"""
import argparse
import glob
import json
import os
import sys
from datetime import datetime

PROJ_DIR = os.path.expanduser("~/.claude/projects")


def find_sessions(project=None):
    pat = os.path.join(PROJ_DIR, project or "*", "*.jsonl")
    out = [(os.path.getmtime(p), p) for p in glob.glob(pat)]
    return sorted(out, reverse=True)


def text_of(content):
    """Rut phan van ban tu content (co the la str hoac list cac block)."""
    if isinstance(content, str):
        return content
    parts = []
    for b in content or []:
        if not isinstance(b, dict):
            continue
        if b.get("type") == "text":
            parts.append(b.get("text", ""))
    return "\n".join(p for p in parts if p.strip())


def tools_of(content):
    """Liet ke cac lan goi cong cu (ten + mo ta ngan), khong lay noi dung day du."""
    out = []
    if isinstance(content, list):
        for b in content:
            if isinstance(b, dict) and b.get("type") == "tool_use":
                inp = b.get("input") or {}
                desc = inp.get("description") or inp.get("file_path") or \
                       inp.get("command", "")[:80] or inp.get("prompt", "")[:80]
                out.append((b.get("name", "?"), str(desc)[:120]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--session")
    ap.add_argument("--project")
    ap.add_argument("--full", action="store_true", help="giu ca ket qua tool")
    ap.add_argument("-o", "--out")
    a = ap.parse_args()

    sessions = find_sessions(a.project)
    if not sessions:
        sys.exit(f"khong thay phien nao trong {PROJ_DIR}")
    if a.list:
        for mt, p in sessions:
            sz = os.path.getsize(p) / 1e6
            print(f"  {datetime.fromtimestamp(mt):%Y-%m-%d %H:%M}  {sz:6.1f} MB  "
                  f"{os.path.basename(p)[:-6]}  [{os.path.basename(os.path.dirname(p))}]")
        return

    path = next((p for _, p in sessions if a.session and a.session in p), None) if a.session \
        else sessions[0][1]
    if path is None:
        sys.exit(f"khong thay phien '{a.session}'")

    lines, n_user, n_asst = [], 0, 0
    with open(path) as fh:
        for raw in fh:
            try:
                d = json.loads(raw)
            except json.JSONDecodeError:
                continue
            t = d.get("type")
            if t not in ("user", "assistant"):
                continue
            msg = d.get("message") or {}
            content = msg.get("content")
            ts = d.get("timestamp", "")[:19].replace("T", " ")
            body = text_of(content)
            if t == "user":
                # bo cac ban ghi ket qua tool (chung cung mang role=user)
                if isinstance(content, list) and all(
                        isinstance(b, dict) and b.get("type") == "tool_result" for b in content):
                    continue
                if not body.strip():
                    continue
                n_user += 1
                lines.append(f"\n---\n\n### Nguoi dung · {ts}\n\n{body}\n")
            else:
                n_asst += 1
                if body.strip():
                    lines.append(f"\n#### Claude · {ts}\n\n{body}\n")
                if not a.full:
                    for name, desc in tools_of(content):
                        lines.append(f"> `{name}` — {desc}\n")

    out = a.out or os.path.join(
        os.getcwd(), f"chat_{os.path.basename(path)[:8]}.md")
    with open(out, "w") as f:
        f.write(f"# Lich su hoi thoai Claude Code\n\n"
                f"Phien: `{os.path.basename(path)[:-6]}`  \n"
                f"Nguon: `{path}`  \n"
                f"Trich luc: {datetime.now():%Y-%m-%d %H:%M}\n")
        f.writelines(lines)
    print(f"{n_user} luot nguoi dung, {n_asst} luot Claude -> {out} "
          f"({os.path.getsize(out)/1024:.0f} KB)")


if __name__ == "__main__":
    main()
