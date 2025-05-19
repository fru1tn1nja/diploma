FROM node:20-alpine AS build
WORKDIR /app
COPY src/frontend/package*.json ./
RUN npm ci
COPY src/frontend .
RUN npm run build

FROM node:20-alpine
WORKDIR /app
ENV NODE_ENV=production
COPY --from=build /app/.next .next
COPY --from=build /app/public ./public
COPY src/frontend/package*.json ./
RUN npm ci --omit=dev
EXPOSE 3000
CMD ["node","node_modules/next/dist/bin/next","start","-p","3000"]