FROM node:14-alpine

RUN apk add --update git

# Clone the eth-netstats repository
RUN git clone https://github.com/cubedro/eth-netstats

WORKDIR /eth-netstats

# Install npm packages
RUN npm install

# Install grunt-cli globally
RUN npm install -g grunt-cli

# Build the full version of ethstats using grunt
RUN grunt all

EXPOSE 3000

# Start the eth-netstats application
CMD ["sh", "-c", "WS_SECRET=mysecret npm start"]