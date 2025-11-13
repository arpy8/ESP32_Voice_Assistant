FROM python:3.12-slim

WORKDIR /code

RUN pip install --no-cache-dir uv

ENV VIRTUAL_ENV=/code/.venv
RUN uv venv $VIRTUAL_ENV
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

COPY pyproject.toml uv.lock* ./

RUN uv sync --frozen
RUN python -m piper.download_voices en_US-libritts_r-medium --data-dir /code/tts_models
COPY . /code

ENV PYTHONPATH=/code/src:/code
ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=1

EXPOSE 7860

CMD ["sh", "-c", "uv run main.py"]