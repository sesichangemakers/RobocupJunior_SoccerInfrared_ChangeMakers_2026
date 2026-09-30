#include "placeholder_page.hpp"

#include "../html_template.hpp"

namespace WebPages {

String renderPlaceholderPage(const char* title, const char* description) {
  String body;
  body.reserve(700);

  body += "<main class=\"panel\">";
  body += "<div class=\"header\">";
  body += "<div class=\"title\">";
  body += title;
  body += "</div>";
  body += HtmlTemplate::backButton("/");
  body += "</div>";
  body += "<p class=\"muted\">";
  body += description;
  body += "</p>";
  body += "</main>";

  return HtmlTemplate::wrapPage(title, body);
}

}  // namespace WebPages
