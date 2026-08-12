# ShapeNet-55 overlap YCB assets

Isaac Sim 6.0의 `/Isaac/Props/YCB/Axis_Aligned/`에 없는 다음 세 YCB geometry를
로컬 USD로 보존한다.

- `001_chips_can.usd`
- `022_windex_bottle.usd`
- `032_knife.usd`

원본은 [YCB Object and Model Set](https://www.ycbbenchmarks.com/)의 공식 S3
archive이며, `scripts/prepare_ycb_overlap_assets.py`가 non-textured STL을 meter,
Z-up USD로 변환한다. Rigid-body mass와 convex-hull collision은
`source/sim/ycb.py`가 스폰할 때 적용한다.

재생성:

```bash
cd /home/frlab/isaac_indy7
~/isaacsim/python.sh scripts/prepare_ycb_overlap_assets.py --force
```
