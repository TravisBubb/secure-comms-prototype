#include "Shell.h"
#include <string.h>
#include <Arduino.h>

Shell::Shell() : _numCommands(0), _lineIdx(0) {}

bool Shell::registerCommand(const char *name, CommandHandler handler)
{
  if (_numCommands >= MAX_COMMANDS) return false;
  _commands[_numCommands++] = {.name = name, .handler = handler};
  return true;
}

void Shell::loop(void)
{
  while (Serial.available())
  {
    char c = Serial.read();

    if (c == '\n')
    {
      _lineBuffer[_lineIdx] = 0;
      if (_lineIdx > 0)
      {
        dispatch(_lineBuffer);
        _lineIdx = 0;
      }
      Serial.print("> ");
    }
    else if (c >= 0x20 && c <= 0x7E)
    {
      // echo char for debugging
      if (_lineIdx < LINE_BUFFER_SIZE - 1)
      {
        _lineBuffer[_lineIdx++] = c;
        Serial.write(c);
      }
      else
      {
        Serial.println("\n[WARN] Line too long, input truncated");
        _lineIdx = 0;
        Serial.print("> ");
      }
    }
  }
}

// Simple argument parser: splits a space-separated string into argv[]
uint8_t Shell::parseArgs(char *args, char *argv[], uint8_t maxArgs)
{
  uint8_t argc = 0;
  if (!args) return 0;

  char *token = strtok(args, " ");
  while (token != nullptr && argc < maxArgs)
  {
    argv[argc++] = token;
    token = strtok(nullptr, " ");
  }

  return argc;
}

void Shell::dispatch(const char *input)
{
  char buffer[128];
  strncpy(buffer, input, sizeof(buffer));
  buffer[sizeof(buffer) - 1] = '\0';

  char *cmd = strtok(buffer, " ");
  char *args = strtok(nullptr, "");

  if (!cmd) return;

  for (uint8_t i = 0; i < _numCommands; i++)
  {
    if (strcmp(cmd, _commands[i].name) == 0)
    {
      char *argv[8];
      uint8_t argc = parseArgs(args, argv, 8);

      _commands[i].handler(argc, argv);
      return;
    }
  }

  Serial.printf("Unknown command: %s\n", cmd);
}