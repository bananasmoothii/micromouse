Pour débugger avec RustRover:

1. Installer l'extension "lsp4ij"
2. setup grâce à [ce tuto](https://github.com/redhat-developer/lsp4ij/blob/main/docs/dap/user-defined-dap/codelldb.md)
   - À la fin il ne faut pas mettre "launch" mais "attach" avec ces paramètres:

```json
{
  "chip": "STM32F446RE",
  "coreConfigs": [
    {
      "programBinary": "${file}"
    }
  ]
}
```

Note: le RTT ne marche pas ici (j'ai essayé avec rttEnabled)