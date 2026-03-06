---
name: browser
description: Open a browser and interact with web pages for testing and investigation. Use when asked to test a web UI, inspect page behavior, take screenshots, check console errors, or debug frontend issues.
argument-hint: [url-or-instruction]
allowed-tools: mcp__chrome-devtools__navigate_page, mcp__chrome-devtools__take_screenshot, mcp__chrome-devtools__take_snapshot, mcp__chrome-devtools__click, mcp__chrome-devtools__fill, mcp__chrome-devtools__fill_form, mcp__chrome-devtools__hover, mcp__chrome-devtools__type_text, mcp__chrome-devtools__press_key, mcp__chrome-devtools__drag, mcp__chrome-devtools__select_page, mcp__chrome-devtools__list_pages, mcp__chrome-devtools__new_page, mcp__chrome-devtools__close_page, mcp__chrome-devtools__wait_for, mcp__chrome-devtools__evaluate_script, mcp__chrome-devtools__list_console_messages, mcp__chrome-devtools__get_console_message, mcp__chrome-devtools__list_network_requests, mcp__chrome-devtools__get_network_request, mcp__chrome-devtools__take_memory_snapshot, mcp__chrome-devtools__lighthouse_audit, mcp__chrome-devtools__handle_dialog, mcp__chrome-devtools__upload_file, mcp__chrome-devtools__emulate, mcp__chrome-devtools__resize_page, mcp__chrome-devtools__performance_start_trace, mcp__chrome-devtools__performance_stop_trace, mcp__chrome-devtools__performance_analyze_insight
---

# Browser Testing Skill

You have access to a Chrome browser via the chrome-devtools MCP server. Use it to test web UIs, investigate rendering issues, capture screenshots, and debug frontend behavior.

## Workflow

1. **Navigate** to the target URL with `navigate_page`
2. **Take a snapshot** with `take_snapshot` to see the page's accessibility tree (element UIDs for interaction)
3. **Interact** using `click`, `fill`, `hover`, `press_key`, `type_text` — all require a `uid` from the snapshot
4. **Inspect** with `evaluate_script` (run JS), `list_console_messages` (check errors), `list_network_requests` (check API calls)
5. **Screenshot** with `take_screenshot` to visually verify state

## Key Patterns

### Navigate and inspect
```
navigate_page(type: "url", url: "http://localhost:8080")
take_snapshot()           # Returns accessibility tree with UIDs
take_screenshot()         # Returns visual screenshot
```

### Interact with elements
After `take_snapshot`, use the `uid` values from the tree:
```
click(uid: "e45")         # Click an element
fill(uid: "e12", value: "hello")  # Fill an input
hover(uid: "e33")         # Hover for tooltips
press_key(key: "Enter")   # Press a key
```

### Debug
```
list_console_messages()                    # Check for JS errors
evaluate_script(function: "() => document.title")  # Run JS
list_network_requests()                    # Check API calls
get_network_request(reqid: 5)              # Inspect specific request/response
```

### Multi-page
```
list_pages()                     # See all tabs
new_page(url: "http://...")      # Open new tab
select_page(pageId: 2)          # Switch to tab
close_page(pageId: 2)           # Close tab
```

## Tips

- Always `take_snapshot` before interacting — you need UIDs to target elements
- Use `includeSnapshot: true` on interaction tools to get an updated snapshot in the response
- Use `wait_for(text: ["some text"])` after navigation or clicks that trigger async loading
- For this project, the GRIB visualisation server runs at `http://localhost:8080`
- Save screenshots to the project root with `filePath` for the user to review
- Check `list_console_messages` after page load to catch JS errors early

## Arguments

If invoked as `/browser http://localhost:8080`, navigate to that URL first. If invoked as `/browser` with no argument, ask what to test.
