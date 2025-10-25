# Monitoring

## Start multiple at once
```shell
curl -X POST localhost:6666/monitor/start \
  -H 'Content-Type: application/json' \
  -d '{"containers":["alpine1","alpine2"],
  "interval":1.0,"csv_dir":null,"stdout":true}' | jq
```
## Start multiple at once (and export results to csv)
```shell
curl -X POST localhost:6666/monitor/start \
  -H 'Content-Type: application/json' \
  -d '{"containers":["alpine1","alpine2"],
  "interval":1.0,"csv_dir":"/experiments/data/docker-stats","stdout":true}' | jq
```

## Status
```shell
curl localhost:6666/monitor/status | jq
```

## Last sample / window totals
```shell
curl 'localhost:6666/monitor/last?container=alpine2' | jq
curl 'localhost:6666/monitor/window?container=alpine2' | jq
```

## Stop all
```shell
curl -X POST "http://localhost:6666/monitor/stop" | jq
```




