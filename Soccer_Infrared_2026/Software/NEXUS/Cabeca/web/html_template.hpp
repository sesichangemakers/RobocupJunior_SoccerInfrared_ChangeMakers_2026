#pragma once

#include <Arduino.h>

namespace HtmlTemplate {

String wrapPage(const char* title,
                const String& bodyHtml,
                const String& extraHead = "",
                const String& extraScript = "");

String backButton(const char* href = "/");

}  // namespace HtmlTemplate
