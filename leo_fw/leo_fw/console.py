# Copyright 2022-2023 Fictionlab sp. z o.o.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from __future__ import annotations

import logging
import time
from collections.abc import Generator
from contextlib import contextmanager
from datetime import UTC, datetime

from rich.console import Console
from rich.live import Live
from rich.logging import RichHandler
from rich.prompt import Confirm
from rich.text import Text

console = Console(log_path=False, log_time=True)

logging.basicConfig(
    level=logging.INFO,
    format="%(message)s",
    datefmt="[%X]",
    handlers=[
        RichHandler(
            console=console,
            show_path=False,
            show_time=True,
            show_level=True,
            markup=True,
        ),
    ],
)


def get_logger(name: str = "leo_fw") -> logging.Logger:
    """
    Get a logger that uses the shared Rich console.

    :param name: The logger name
    :type name: str
    :return: Logger writing to the shared console
    :rtype: logging.Logger
    """
    return logging.getLogger(name)


_log = get_logger()


def _get_log_prefix() -> Text:
    """Get the standard log message prefix (timestamp and level) as Text."""
    return Text.assemble(
        (datetime.now(tz=UTC).astimezone().strftime("[%X]"), "log.time"),
        " ",
        ("INFO    ", "logging.level.info"),
        " ",
    )


@contextmanager
def log_step(
    label: str,
    success_text: str = "OK",
    failure_text: str = "FAILED",
) -> Generator[None, None, None]:
    """
    Render a step line and emit a log record once the step is finished.

    :param label: The description text shown while the step is in progress
    :type label: str
    :param success_text: Text shown when the step completes successfully
    :type success_text: str
    :param failure_text: Text shown when the step raises an exception
    :type failure_text: str
    """
    start = time.perf_counter()

    live = Live(
        Text.assemble(_get_log_prefix(), f"{label}..."), console=console, transient=True
    )
    live.start()

    try:
        yield
    except Exception:
        live.stop()
        elapsed_ms = (time.perf_counter() - start) * 1000
        _log.info("%s... [red]%s[/red] (%.0f ms)", label, failure_text, elapsed_ms)
        raise

    live.stop()
    elapsed_ms = (time.perf_counter() - start) * 1000
    _log.info("%s... [green]%s[/green] (%.0f ms)", label, success_text, elapsed_ms)


def get_confirmation_prompt(prompt: str, default: bool = False) -> bool:
    """
    Display a confirmation prompt styled consistently with the logging output.

    :param prompt: The question to display
    :type prompt: str
    :param default: The answer used when the user just hits <Enter>
    :type default: bool
    :return: True if the user confirmed, False otherwise
    :rtype: bool
    """
    return Confirm.ask(
        Text.assemble(_get_log_prefix(), prompt), default=default, console=console
    )


def report_results(logger: logging.Logger, results: list[tuple[str, bool]]) -> int:
    """
    Report the final test results and return a process exit code.

    :param logger: Logger used to write the summary
    :type logger: logging.Logger
    :param results: Test names paired with their pass/fail status
    :type results: list[tuple[str, bool]]
    :return: 0 if every check passed, 1 otherwise
    :rtype: int
    """
    failed = [name for name, passed in results if not passed]

    if failed:
        logger.error(
            "Finished with %d failing check(s): %s.", len(failed), "; ".join(failed)
        )
        return 1

    logger.info("Finished successfully. %d check(s) passed.", len(results))
    return 0
