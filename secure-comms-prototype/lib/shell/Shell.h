#ifndef SHELL_H
#define SHELL_H

#include <stdint.h>
#include <stddef.h>

using CommandHandler = void (*)(uint8_t argc, char *argv[]);

static constexpr size_t LINE_BUFFER_SIZE = 128;

struct Command
{
  const char *name;
  CommandHandler handler;
};

class Shell
{
public:
  Shell();
  bool registerCommand(const char *name, CommandHandler handler);
  void loop(void);

private:
  static constexpr uint8_t MAX_COMMANDS = 16;
  Command _commands[MAX_COMMANDS];
  uint8_t _numCommands;
  char _lineBuffer[LINE_BUFFER_SIZE];
  size_t _lineIdx;

  uint8_t parseArgs(char *args, char *argv[], uint8_t maxArgs);
  void dispatch(const char *input);
};

#endif // SHELL_H