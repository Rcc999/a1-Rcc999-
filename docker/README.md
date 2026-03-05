# Docker build (Ubuntu)

Build the image:

```bash
docker build -f docker/Dockerfile.ubuntu -t cmm-a1-ubuntu .
```

Run a shell in the image:

```bash
docker run --rm -it cmm-a1-ubuntu
```

Note: This image builds the project during `docker build`. It includes GUI build dependencies but does not run the GUI.
