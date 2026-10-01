// The theme chosen on this page (system, light, or dark), set before anything is drawn, so a dark
// page never flashes light. A plain script, loaded in the page's head: the modules run only after
// the page is read. Its own file, as the page runs no inline script.
try {
  const theme = localStorage.getItem("pitwall.theme");
  if (theme === "light" || theme === "dark") document.documentElement.dataset.theme = theme;
} catch (error) {
  // Storage is off in this browser: the system's theme.
}
