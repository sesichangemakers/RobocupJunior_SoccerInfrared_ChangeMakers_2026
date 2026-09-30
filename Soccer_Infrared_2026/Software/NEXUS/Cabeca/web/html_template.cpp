#include "html_template.hpp"

namespace HtmlTemplate {

String wrapPage(const char* title,
                const String& bodyHtml,
                const String& extraHead,
                const String& extraScript) {
  String html;
  html.reserve(1200 + bodyHtml.length() + extraHead.length() + extraScript.length());

  html += "<!doctype html><html lang=\"pt-BR\"><head>";
  html += "<meta charset=\"utf-8\"/>";
  html += "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\"/>";
  html += "<title>";
  html += title;
  html += "</title>";

  html += "<style>";
  html += ":root{--bg1:#180606;--bg2:#040404;--card:#0f0b0b;--ink:#f3f3f3;--muted:#c7a8a8;--line:#4b1a1a;--btn:#d91111;--btn2:#8f0b0b;}";
  html += "*{box-sizing:border-box;}";
  html += "body{margin:0;min-height:100vh;padding:16px;display:grid;place-items:center;font-family:'Segoe UI',Tahoma,sans-serif;color:var(--ink);background:radial-gradient(circle at 18% 10%,#4a0e0e 0%,#180606 28%,#070707 62%,#020202 100%);}";
  html += ".panel{width:min(980px,100%);background:var(--card);border:1px solid var(--line);border-radius:16px;box-shadow:0 14px 30px rgba(0,0,0,.45);padding:18px;}";
  html += ".header{display:flex;justify-content:space-between;align-items:center;gap:12px;flex-wrap:wrap;margin-bottom:12px;}";
  html += ".title{font-size:1.2rem;font-weight:700;}";
  html += ".muted{color:var(--muted);font-size:.92rem;}";
  html += ".btn{display:inline-block;border:0;border-radius:10px;padding:10px 14px;text-decoration:none;font-weight:700;color:#fff;background:linear-gradient(180deg,var(--btn),var(--btn2));box-shadow:0 6px 14px rgba(0,0,0,.35);}";
  html += ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:12px;}";
  html += ".card{display:block;text-decoration:none;color:inherit;border:1px solid var(--line);border-radius:12px;padding:14px;background:linear-gradient(180deg,#1a0d0d,#110909);}";
  html += ".card h3{margin:0 0 6px;font-size:1rem;}";
  html += ".card p{margin:0;color:var(--muted);font-size:.9rem;}";
  html += extraHead;
  html += "</style></head><body>";
  html += bodyHtml;
  html += extraScript;
  html += "</body></html>";

  return html;
}

String backButton(const char* href) {
  String b;
  b.reserve(64);
  b += "<a class=\"btn\" href=\"";
  b += href;
  b += "\">Voltar</a>";
  return b;
}

}  // namespace HtmlTemplate
