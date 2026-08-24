"""移設済みBLDCアプリを検査し、常駐ブートローダー用CRC32C metadataを生成する。"""
from __future__ import annotations
import argparse, struct
from pathlib import Path
APP_BASE=0x08004000;APP_SIZE=0x1B000;MAGIC=0x3157464F
def crc32c(data:bytes)->int:
 c=0xFFFFFFFF
 for value in data:
  c^=value
  for _ in range(8):c=(c>>1)^(0x82F63B78 if c&1 else 0)
 return (~c)&0xFFFFFFFF
def main()->None:
 p=argparse.ArgumentParser();p.add_argument("image",type=Path);p.add_argument("output",type=Path);p.add_argument("--generation",type=int,default=1);a=p.parse_args();image=a.image.read_bytes()
 if not 8<=len(image)<=APP_SIZE:raise ValueError(f"invalid image size: {len(image)}")
 sp,rh=struct.unpack_from("<II",image);valid_sp=0x20000000<=sp<=0x20008000 or 0x10000000<=sp<=0x10002000
 if not valid_sp or sp%8 or not rh&1 or not APP_BASE<=(rh&~1)<APP_BASE+len(image):raise ValueError(f"invalid vector sp=0x{sp:08X} reset=0x{rh:08X}")
 record=struct.pack("<IHHIIIIII",MAGIC,1,36,a.generation,4,0,APP_BASE,len(image),crc32c(image));metadata=record+struct.pack("<I",crc32c(record));a.output.write_bytes(metadata)
 print(f"size={len(image)} image_crc32c=0x{crc32c(image):08X} metadata={a.output}")
if __name__=="__main__":main()
