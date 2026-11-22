# Quorum Explorer (block explorer web UI on port 25000), built from source.
# The published image consensys/quorum-explorer is gone from Docker Hub; the
# source repo is archived. The source comes in as a named build context pinned
# to the commit of the image we used (QUORUM_EXPLORER_VERSION in the blockchain .env):
#   --build-context source=https://github.com/Consensys-inc-archive/quorum-explorer.git#4f601913bd008164aeba0d06d8724142f4b9f52d
#
# Same three stages as the upstream Dockerfile, with Node pinned to 18 (the LTS
# when that commit was built): Next.js 12.1 does not build on later Node.
ARG NODE_IMAGE=node:18-alpine

FROM ${NODE_IMAGE} AS dependencies
WORKDIR /app
COPY --from=source package.json package-lock.json ./
RUN apk add --no-cache --virtual .gyp python3 make g++ \
    && npm ci \
    && apk del .gyp

FROM ${NODE_IMAGE} AS builder
WORKDIR /app
COPY --from=source . .
COPY --from=dependencies /app/node_modules ./node_modules
RUN npm run build

FROM ${NODE_IMAGE}
ENV NODE_ENV=production
WORKDIR /app
COPY --from=builder /app/.env.production /app/next.config.js /app/package.json ./
COPY --from=builder /app/public ./public
COPY --from=builder /app/.next ./.next
COPY --from=builder /app/node_modules ./node_modules
EXPOSE 25000
CMD ["npm", "run", "start"]
