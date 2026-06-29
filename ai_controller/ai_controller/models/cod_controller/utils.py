import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision
from torchvision import models
import math
import numpy as np
from einops import rearrange, reduce, repeat
from torchsummary import summary
import xml.etree.ElementTree as ET
import numpy as np
import os
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from tqdm import tqdm

import torch
from torchvision import ops
import torch.nn.functional as F
import torch.optim as optim
from einops import *
from collections import OrderedDict 
from torchvision import transforms
from torchvision.transforms.functional import resized_crop


class ResNetFeats(nn.Module):
    def __init__(self, out_dim=256, output_raw=False, drop_dim=1, use_resnet18=False, pretrained=False):
        super(ResNetFeats, self).__init__()
        print('pretrain', pretrained)
        torchvision_version = torchvision.__version__
        if not torchvision_version.split('+')[0] in '0.9.1':
            if pretrained and use_resnet18:
                resnet = models.resnet18(
                    weights=models.ResNet18_Weights.DEFAULT)
            elif pretrained and not use_resnet18:
                resnet = models.resnet50(
                    weights=models.ResNet50_Weights.DEFAULT)
            elif not pretrained and use_resnet18:
                resnet = models.resnet18(weights=None)
            elif not pretrained and not use_resnet18:
                resnet = models.resnet50(weights=None)
        else:
            resnet = models.resnet18(
                pretrained=pretrained) if use_resnet18 else models.resnet50(pretrained=pretrained)
        print("---- Backbone summary of resnet ----")
        self._features = nn.Sequential(*list(resnet.children())[:-drop_dim])
        # summary(self._features)
        self._output_raw = output_raw
        self._out_dim = 512 if use_resnet18 else 2048
        self._out_dim = int(self._out_dim / 2 ** (drop_dim - 2)
                            ) if drop_dim >= 2 else self._out_dim
        if not output_raw:
            self._nn_out = nn.Sequential(nn.Conv2d(self._out_dim, out_dim, 1), nn.BatchNorm2d(
                out_dim), nn.ReLU(inplace=True), nn.Conv2d(out_dim, out_dim, 1))
            self._out_dim = out_dim

    def forward(self, inputs, depth=None):
        reshaped = len(inputs.shape) == 5
        x = inputs.reshape(
            (-1, inputs.shape[-3], inputs.shape[-2], inputs.shape[-1])).float()

        out = self._features(x)
        out = self._nn_out(out) if not self._output_raw else out
        NH, NW = out.shape[-2:]

        # reshape to proper dimension
        out = out.reshape(
            (inputs.shape[0], inputs.shape[1], self._out_dim, NH, NW)) if reshaped else out
        if NH * NW == 1:
            out = out[:, :, :, 0, 0] if reshaped else out[:, :, 0, 0]
        return out

    @property
    def dim(self):
        return self._out_dim


class VGGFeats(nn.Module):
    def __init__(self, out_dim=512):
        super().__init__()
        self._vgg = nn.Sequential(
            *list(models.vgg16(pretrained=True).features.children())[:9])

        def _conv_block(in_dim, out_dim, N):
            # pool and coord conv
            cc = CoordConv(in_dim, out_dim, 2, stride=2, padding=0)
            n, a = nn.BatchNorm2d(out_dim), nn.ReLU(inplace=True)
            ops = [cc, n, a]

            for _ in range(N):
                c = nn.Conv2d(out_dim, out_dim, 3, padding=1)
                n, a = nn.BatchNorm2d(out_dim), nn.ReLU(inplace=True)
                ops.extend([c, n, a])
            return ops

        self._v1 = nn.Sequential(*_conv_block(128, 256, 3))
        self._v2 = nn.Sequential(*_conv_block(256, 512, 3))
        self._pool = nn.AdaptiveAvgPool2d(1)
        self._out_dim = out_dim
        self._linear = nn.Sequential(
            nn.Linear(512, out_dim), nn.ReLU(inplace=True))

    def forward(self, x, depth=None):
        reshaped = len(x.shape) == 5
        in_x = x.reshape(
            (x.shape[0] * x.shape[1], x.shape[2], x.shape[3], x.shape[4])) if reshaped else x
        vis_feat = self._pool(self._v2(self._v1(self._vgg(in_x))))
        out_feat = self._linear(vis_feat[:, :, 0, 0])
        out_feat = out_feat.view(
            (x.shape[0], x.shape[1], self._out_dim)) if reshaped else out_feat
        return out_feat

    @property
    def dim(self):
        return self._out_dim


class TemporalPositionalEncoding(nn.Module):
    """
    Modified PositionalEncoding from Pytorch Seq2Seq Documentation
    source: https://pytorch.org/tutorials/beginner/transformer_tutorial.html
    """

    def __init__(self, d_model, dropout=0.1, max_len=3000):
        super().__init__()
        self.dropout = nn.Dropout(p=dropout)

        pe = torch.zeros(max_len, d_model)
        position = torch.arange(
            0, max_len, dtype=torch.float).unsqueeze(1)  # (max_len, 1)
        div_term = torch.exp(torch.arange(
            0, d_model, 2).float() * (-np.log(10000.0) / d_model))
        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term)
        pe = pe.unsqueeze(0).transpose(1, 2)
        print("Position embedding shape: {} \n".format(pe.shape))
        self.register_buffer('pe', pe)

    def forward(self, x):
        assert len(x.shape) >= 3, "x requires at least 3 dims! (B, C, ..)"
        old_shape = x.shape
        x = x.reshape((x.shape[0], x.shape[1], -1))  # B,d,T*H*W
        x = x + self.pe[:, :, :x.shape[-1]]
        return self.dropout(x).reshape(old_shape)


class NonLocalLayer(nn.Module):
    def __init__(self, in_dim, out_dim, feedforward_dim=512, dropout=0, temperature=None, causal=False, n_heads=1):
        super().__init__()
        assert feedforward_dim % n_heads == 0, "n_heads must evenly divide feedforward_dim"
        self._n_heads = n_heads
        self._temperature = temperature if temperature is not None else np.sqrt(
            in_dim)
        self._K = nn.Conv3d(in_dim, feedforward_dim, 1, bias=False)
        self._V = nn.Conv3d(in_dim, feedforward_dim, 1, bias=False)
        self._Q = nn.Conv3d(in_dim, feedforward_dim, 1, bias=False)
        self._out = nn.Conv3d(feedforward_dim, out_dim, 1)
        self._a1, self._drop1 = nn.ReLU(
            inplace=dropout == 0), nn.Dropout3d(dropout)
        self._norm = nn.BatchNorm3d(out_dim)
        self._causal = causal
        self._skip = out_dim == in_dim

    def forward(self, inputs):
        # inputs shape: B, d, T, H, W
        K, Q, V = self._K(inputs), self._Q(inputs), self._V(inputs)
        B, C, T, H, W = K.shape
        K, Q, V = [
            rearrange(t, 'B (head channel) T H W -> B head channel (T H W)',
                      head=self._n_heads, channel=int(C / self._n_heads))
            for t in (K, Q, V)]
        KQ = torch.einsum('bnci,bncj->bnij', K, Q) / \
            self._temperature  # B, heads, THW, THW
        if self._causal:
            mask = torch.tril(torch.ones((T, T))).to(KQ.device)
            mask = mask.repeat_interleave(
                H*W, 0).repeat_interleave(H*W, 1)  # -> (T*H*W, T*H*W)
            # -> (1, 1, T*H*W, T*H*W)
            KQ = KQ + torch.log(mask).unsqueeze(0).unsqueeze(0)
        attn = F.softmax(KQ, 3)
        V = torch.einsum('bncj,bnij->bnci', V, attn).reshape((B, C, T, H, W))
        out = inputs + self._drop1(self._a1(self._out(V))
                                   ) if self._skip else self._drop1(self._a1(self._out(V)))
        return self._norm(out)


class TempConvLayer(nn.Module):
    def __init__(self, in_dim, out_dim, feedforward_dim=512, dropout=0, temperature=None, causal=False, n_heads=None, k=5):
        super().__init__()
        self._causal, self._skip = causal, out_dim == in_dim
        frame_pad = int(k // 2)
        self._pad = k - 1 if causal else frame_pad
        self._c1, self._a1 = nn.Conv3d(
            in_dim, feedforward_dim, 1), nn.ReLU(inplace=True)
        self._c2, self._a2 = nn.Conv3d(feedforward_dim, feedforward_dim, k, padding=(
            self._pad, frame_pad, frame_pad)), nn.ReLU(inplace=True)
        self._c3, self._a3 = nn.Conv3d(
            feedforward_dim, out_dim, 1), nn.ReLU(inplace=dropout == 0)
        self._drop = nn.Dropout3d(dropout)
        self._norm = nn.BatchNorm3d(out_dim)

    def forward(self, inputs):
        downsized = self._a1(self._c1(inputs))
        ff = self._a2(self._c2(downsized))
        ff = ff[:, :, :-self._pad] if self._causal else ff
        upsized = self._drop(self._a3(self._c3(ff)))
        out = inputs + upsized if self._skip else upsized
        return self._norm(out)

# -------------- Data Utils -------------------
def parse_annotation(annotation_path, image_dir, img_size):
    '''
    Traverse the xml tree, get the annotations, and resize them to the scaled image size
    '''
    img_h, img_w = img_size

    with open(annotation_path, "r") as f:
        tree = ET.parse(f)

    root = tree.getroot()

    img_paths = []
    gt_boxes_all = []
    gt_classes_all = []
    # get image paths
    for object_ in root.findall('image'):
        img_path = os.path.join(image_dir, object_.get("name"))
        img_paths.append(img_path)

        # get raw image size
        orig_w = int(object_.get("width"))
        orig_h = int(object_.get("height"))

        # get bboxes and their labels
        groundtruth_boxes = []
        groundtruth_classes = []
        for box_ in object_.findall('box'):
            xmin = float(box_.get("xtl"))
            ymin = float(box_.get("ytl"))
            xmax = float(box_.get("xbr"))
            ymax = float(box_.get("ybr"))

            # rescale bboxes
            bbox = torch.Tensor([xmin, ymin, xmax, ymax])
            bbox[[0, 2]] = bbox[[0, 2]] * img_w/orig_w
            bbox[[1, 3]] = bbox[[1, 3]] * img_h/orig_h

            groundtruth_boxes.append(bbox.tolist())

            # get labels
            label = box_.get("label")
            groundtruth_classes.append(label)

        gt_boxes_all.append(torch.Tensor(groundtruth_boxes))
        gt_classes_all.append(groundtruth_classes)

    return gt_boxes_all, gt_classes_all, img_paths

# -------------- Prepocessing utils ----------------
def calc_gt_offsets(pos_anc_coords, gt_bbox_mapping):
    pos_anc_coords = ops.box_convert(
        pos_anc_coords, in_fmt='xyxy', out_fmt='cxcywh')
    gt_bbox_mapping = ops.box_convert(
        gt_bbox_mapping, in_fmt='xyxy', out_fmt='cxcywh')

    gt_cx, gt_cy, gt_w, gt_h = gt_bbox_mapping[:, 0], gt_bbox_mapping[:,
                                                                      1], gt_bbox_mapping[:, 2], gt_bbox_mapping[:, 3]
    anc_cx, anc_cy, anc_w, anc_h = pos_anc_coords[:,
                                                  0], pos_anc_coords[:, 1], pos_anc_coords[:, 2], pos_anc_coords[:, 3]

    tx_ = (gt_cx - anc_cx)/anc_w
    ty_ = (gt_cy - anc_cy)/anc_h
    tw_ = torch.log(gt_w / anc_w)
    th_ = torch.log(gt_h / anc_h)

    return torch.stack([tx_, ty_, tw_, th_], dim=-1)


def gen_anc_centers(out_size, x_offset, y_offset, step=1):
    out_h, out_w = out_size
    
    # 2.0 real-world dataset
    # 1.5 sim dataset
    anc_pts_x = torch.arange(0, out_w, step) + x_offset # 1.5
    anc_pts_y = torch.arange(0, out_h, step) + y_offset

    return anc_pts_x, anc_pts_y


def project_bboxes(bboxes, width_scale_factor, height_scale_factor, mode='a2p'):
    assert mode in ['a2p', 'p2a']

    batch_size = bboxes.size(dim=0)
    proj_bboxes = bboxes.clone().reshape(batch_size, -1, 4).to(bboxes.get_device())
    invalid_bbox_mask = (proj_bboxes == -1)  # indicating padded bboxes

    if mode == 'a2p':
        # activation map to pixel image

        proj_bboxes[:, :, [0, 2]] = (proj_bboxes[:, :, [
                                     0, 2]] * width_scale_factor).clone().detach().to(bboxes.device).float()

        proj_bboxes[:, :, [1, 3]] = (proj_bboxes[:, :, [
                                     1, 3]] * height_scale_factor).clone().detach().to(bboxes.device).float()
    else:
        # pixel image to activation map
        proj_bboxes[:, :, [0, 2]] = (proj_bboxes[:, :, [
                                     0, 2]] / width_scale_factor).clone().detach().to(bboxes.device).float()
        proj_bboxes[:, :, [1, 3]] = (proj_bboxes[:, :, [
                                     1, 3]] / height_scale_factor).clone().detach().to(bboxes.device).float()
    # fill padded bboxes back with -1
    proj_bboxes.masked_fill_(invalid_bbox_mask, -1)

    return proj_bboxes


def generate_proposals(anchors, offsets):

    # change format of the anchor boxes from 'xyxy' to 'cxcywh'
    anchors = ops.box_convert(anchors, in_fmt='xyxy', out_fmt='cxcywh')

    # apply offsets to anchors to create proposals
    proposals_ = torch.zeros_like(anchors)
    proposals_[:, 0] = anchors[:, 0] + offsets[:, 0]*anchors[:, 2]
    proposals_[:, 1] = anchors[:, 1] + offsets[:, 1]*anchors[:, 3]
    proposals_[:, 2] = anchors[:, 2] * torch.exp(offsets[:, 2])
    proposals_[:, 3] = anchors[:, 3] * torch.exp(offsets[:, 3])

    # change format of proposals back from 'cxcywh' to 'xyxy'
    proposals = ops.box_convert(proposals_, in_fmt='cxcywh', out_fmt='xyxy')
    return proposals
    # """
    # Generate proposals for a batch of anchor boxes and offsets.

    # Args:
    #     anchors (Tensor): Batch of anchor boxes in 'xyxy' format.
    #     offsets (Tensor): Batch of offsets to be applied to anchors.

    # Returns:
    #     proposals (Tensor): Batch of proposals in 'xyxy' format.
    # """
    # # Change format of the anchor boxes from 'xyxy' to 'cxcywh'
    # anchors = ops.box_convert(anchors, in_fmt='xyxy', out_fmt='cxcywh')

    # # Apply offsets to anchors to create proposals
    # proposals_ = torch.zeros_like(anchors)
    # proposals_[:, :, 0] = anchors[:, :, 0] + \
    #     offsets[:, :, 0] * anchors[:, :, 2]
    # proposals_[:, :, 1] = anchors[:, :, 1] + \
    #     offsets[:, :, 1] * anchors[:, :, 3]
    # proposals_[:, :, 2] = anchors[:, :, 2] * torch.exp(offsets[:, :, 2])
    # proposals_[:, :, 3] = anchors[:, :, 3] * torch.exp(offsets[:, :, 3])

    # # Change format of proposals back from 'cxcywh' to 'xyxy'
    # proposals = ops.box_convert(proposals_, in_fmt='cxcywh', out_fmt='xyxy')

    # return proposals


def gen_anc_base_optimized(anc_pts_x, anc_pts_y, anc_scales, anc_ratios, out_size):
    n_anc_boxes = len(anc_scales) * len(anc_ratios)
    batch_size = anc_pts_x.size(0)
    anc_base = torch.zeros(batch_size, anc_pts_x.size(
        1), anc_pts_y.size(1), n_anc_boxes, 4)

    # Precompute anchor sizes based on scales and ratios
    anchor_sizes = [(scale * ratio, scale)
                    for scale in anc_scales for ratio in anc_ratios]

    for batch_idx in range(batch_size):
        for ix in range(anc_pts_x.size(1)):
            for jx in range(anc_pts_y.size(1)):
                xc = anc_pts_x[batch_idx, ix]
                yc = anc_pts_y[batch_idx, jx]

                # Compute anchor coordinates in a vectorized manner
                xmin = xc - 0.5 * torch.tensor(anchor_sizes)[:, 0]
                ymin = yc - 0.5 * torch.tensor(anchor_sizes)[:, 1]
                xmax = xc + 0.5 * torch.tensor(anchor_sizes)[:, 0]
                ymax = yc + 0.5 * torch.tensor(anchor_sizes)[:, 1]

                # Stack the anchor boxes along the last dimension
                anc_boxes = torch.stack([xmin, ymin, xmax, ymax], dim=-1)

                # Clip anchor boxes to the image size
                anc_boxes = ops.clip_boxes_to_image(anc_boxes, size=out_size)

                # Store the computed anchor boxes in the result tensor
                anc_base[batch_idx, ix, jx, :] = anc_boxes

    return anc_base


def gen_anc_base(anc_pts_x, anc_pts_y, anc_scales, anc_ratios, out_size):
    n_anc_boxes = len(anc_scales) * len(anc_ratios)
    anc_base = torch.zeros(1, anc_pts_x.size(dim=0), anc_pts_y.size(
        dim=0), n_anc_boxes, 4)  # shape - [1, Hmap, Wmap, n_anchor_boxes, 4]

    for ix, xc in enumerate(anc_pts_x):
        for jx, yc in enumerate(anc_pts_y):
            anc_boxes = torch.zeros((n_anc_boxes, 4))
            c = 0
            for i, scale in enumerate(anc_scales):
                for j, ratio in enumerate(anc_ratios):
                    w = scale * ratio
                    h = scale

                    xmin = xc - w / 2
                    ymin = yc - h / 2
                    xmax = xc + w / 2
                    ymax = yc + h / 2

                    anc_boxes[c, :] = torch.Tensor([xmin, ymin, xmax, ymax])
                    c += 1

            anc_base[:, ix, jx, :] = ops.clip_boxes_to_image(
                anc_boxes, size=out_size)

    return anc_base


def get_iou_mat(batch_size, anc_boxes_all, gt_bboxes_all):

    # flatten anchor boxes
    anc_boxes_flat = anc_boxes_all.reshape(batch_size, -1, 4)
    # get total anchor boxes for a single image
    tot_anc_boxes = anc_boxes_flat.size(dim=1)

    # create a placeholder to compute IoUs amongst the boxes
    ious_mat = torch.zeros(
        (batch_size, tot_anc_boxes, gt_bboxes_all.size(dim=1)))

    # compute IoU of the anc boxes with the gt boxes for all the images
    for i in range(batch_size):
        gt_bboxes = gt_bboxes_all[i]
        anc_boxes = anc_boxes_flat[i]
        ious_mat[i, :] = ops.box_iou(anc_boxes, gt_bboxes)

    return ious_mat


def get_req_anchors(anc_boxes_all, gt_bboxes_all, gt_classes_all, pos_thresh=0.7, neg_thresh=0.2):
    '''
    Prepare necessary data required for training

    Input
    ------
    anc_boxes_all - torch.Tensor of shape (B, w_amap, h_amap, n_anchor_boxes, 4)
        all anchor boxes for a batch of images
    gt_bboxes_all - torch.Tensor of shape (B, max_objects, 4)
        padded ground truth boxes for a batch of images
    gt_classes_all - torch.Tensor of shape (B, max_objects)
        padded ground truth classes for a batch of images

    Returns
    ---------
    positive_anc_ind -  torch.Tensor of shape (n_pos,)
        flattened positive indices for all the images in the batch
    negative_anc_ind - torch.Tensor of shape (n_pos,)
        flattened positive indices for all the images in the batch
    GT_conf_scores - torch.Tensor of shape (n_pos,), IoU scores of +ve anchors
    GT_offsets -  torch.Tensor of shape (n_pos, 4),
        offsets between +ve anchors and their corresponding ground truth boxes
    GT_class_pos - torch.Tensor of shape (n_pos,)
        mapped classes of +ve anchors
    positive_anc_coords - (n_pos, 4) coords of +ve anchors (for visualization)
    negative_anc_coords - (n_pos, 4) coords of -ve anchors (for visualization)
    positive_anc_ind_sep - list of indices to keep track of +ve anchors
    '''
    # get the size and shape parameters
    B, w_amap, h_amap, A, _ = anc_boxes_all.shape
    N = gt_bboxes_all.shape[1]  # max number of groundtruth bboxes in a batch

    # get total number of anchor boxes in a single image
    tot_anc_boxes = A * w_amap * h_amap

    # get the iou matrix which contains iou of every anchor box
    # against all the groundtruth bboxes in an image
    iou_mat = get_iou_mat(B, anc_boxes_all, gt_bboxes_all)

    # for every groundtruth bbox in an image, find the iou
    # with the anchor box which it overlaps the most
    max_iou_per_gt_box, _ = iou_mat.max(dim=1, keepdim=True)

    # get positive anchor boxes

    # condition 1: the anchor box with the max iou for every gt bbox
    positive_anc_mask = torch.logical_and(
        iou_mat == max_iou_per_gt_box, max_iou_per_gt_box > pos_thresh)
    # condition 2: anchor boxes with iou above a threshold with any of the gt bboxes
    positive_anc_mask = torch.logical_or(
        positive_anc_mask, iou_mat > pos_thresh)

    positive_anc_ind_sep = torch.where(positive_anc_mask)[
        0]  # get separate indices in the batch
    # combine all the batches and get the idxs of the +ve anchor boxes
    positive_anc_mask = positive_anc_mask.flatten(start_dim=0, end_dim=1)
    positive_anc_ind = torch.where(positive_anc_mask)[0]

    # for every anchor box, get the iou and the idx of the
    # gt bbox it overlaps with the most
    max_iou_per_anc, max_iou_per_anc_ind = iou_mat.max(dim=-1)
    max_iou_per_anc = max_iou_per_anc.flatten(start_dim=0, end_dim=1)

    # get iou scores of the +ve anchor boxes
    GT_conf_scores = max_iou_per_anc[positive_anc_ind]

    # get gt classes of the +ve anchor boxes

    # expand gt classes to map against every anchor box
    gt_classes_expand = gt_classes_all.view(
        B, 1, N).expand(B, tot_anc_boxes, N)
    # for every anchor box, consider only the class of the gt bbox it overlaps with the most
    GT_class = torch.gather(gt_classes_expand, -1,
                            max_iou_per_anc_ind.unsqueeze(-1).to(gt_classes_expand.get_device())).squeeze(-1)
    # combine all the batches and get the mapped classes of the +ve anchor boxes
    GT_class = GT_class.flatten(start_dim=0, end_dim=1)
    GT_class_pos = GT_class[positive_anc_ind]

    # get gt bbox coordinates of the +ve anchor boxes

    # expand all the gt bboxes to map against every anchor box
    gt_bboxes_expand = gt_bboxes_all.view(
        B, 1, N, 4).expand(B, tot_anc_boxes, N, 4)
    # for every anchor box, consider only the coordinates of the gt bbox it overlaps with the most
    GT_bboxes = torch.gather(
        gt_bboxes_expand, -2, max_iou_per_anc_ind.reshape(B, tot_anc_boxes, 1, 1).repeat(1, 1, 1, 4).to(gt_classes_expand.get_device()))
    # combine all the batches and get the mapped gt bbox coordinates of the +ve anchor boxes
    GT_bboxes = GT_bboxes.flatten(start_dim=0, end_dim=2)
    GT_bboxes_pos = GT_bboxes[positive_anc_ind]

    # get coordinates of +ve anc boxes
    anc_boxes_flat = anc_boxes_all.flatten(
        start_dim=0, end_dim=-2)  # flatten all the anchor boxes
    positive_anc_coords = anc_boxes_flat[positive_anc_ind]

    # calculate gt offsets
    GT_offsets = calc_gt_offsets(
        positive_anc_coords.to(
            gt_classes_expand.get_device()),
        GT_bboxes_pos)

    # get -ve anchors

    # condition: select the anchor boxes with max iou less than the threshold
    negative_anc_mask = (max_iou_per_anc < neg_thresh)
    negative_anc_ind = torch.where(negative_anc_mask)[0]
    # sample -ve samples to match the +ve samples
    negative_anc_ind = negative_anc_ind[torch.randint(
        0, negative_anc_ind.shape[0], (positive_anc_ind.shape[0],))]
    negative_anc_coords = anc_boxes_flat[negative_anc_ind]

    return positive_anc_ind, negative_anc_ind, GT_conf_scores, GT_offsets, GT_class_pos, \
        positive_anc_coords, negative_anc_coords, positive_anc_ind_sep

# # -------------- Visualization utils ----------------
def display_img(img_data, fig, axes):
    for i, img in enumerate(img_data):
        if type(img) == torch.Tensor:
            img = img.permute(1, 2, 0).numpy()
        axes[i].imshow(img)

    return fig, axes


def display_bbox(bboxes, fig, ax, classes=None, in_format='xyxy', color='y', line_width=3):
    if type(bboxes) == np.ndarray:
        bboxes = torch.from_numpy(bboxes)
    if classes:
        assert len(bboxes) == len(classes)
    # convert boxes to xywh format
    bboxes = ops.box_convert(bboxes, in_fmt=in_format, out_fmt='xywh')
    c = 0
    for box in bboxes:
        x, y, w, h = box.numpy()
        # display bounding box
        rect = patches.Rectangle(
            (x, y), w, h, linewidth=line_width, edgecolor=color, facecolor='none')
        ax.add_patch(rect)
        # display category
        if classes:
            if classes[c] == 'pad':
                continue
            ax.text(x + 5, y + 20, classes[c],
                    bbox=dict(facecolor='yellow', alpha=0.5))
        c += 1

    return fig, ax


def display_grid(x_points, y_points, fig, ax, special_point=None):
    # plot grid
    for x in x_points:
        for y in y_points:
            ax.scatter(x, y, color="w", marker='+')

    # plot a special point we want to emphasize on the grid
    if special_point:
        x, y = special_point
        ax.scatter(x, y, color="red", marker='+')

    return fig, ax

# -------------- VideoImitation Model utils ----------------
def get_model(name):
    if name == 'resnet':
        return ResNetFeats
    elif name == 'vgg':
        return VGGFeats
    else:
        raise NotImplementedError


def get_class_activation_map(feature_map: np.array, input_image: np.array):
    # Resize the feature map to match the input image size
    resized_feature_map = cv2.resize(
        feature_map, (input_image.shape[0], input_image.shape[1]))

    # Compute the average over the feature maps
    if feature_map.shape[2] != 1:
        cam = np.mean(resized_feature_map, axis=2)
    else:
        cam = resized_feature_map
    cam_img = (cam - np.min(cam)) / (np.max(cam) - np.min(cam))
    cam_img = np.uint8(255 * cam_img)
    cv2.imwrite("cam_no_color_jet.png", cam_img)

    # Add the CAM to the input image as an overlay
    output_image = cv2.applyColorMap(cam_img, cv2.COLORMAP_JET)
    output_image = cv2.addWeighted(input_image, 0.5, output_image, 0.5, 0)

    cv2.imwrite("cam.png", output_image)

    return cam_img, output_image



def build_tvf_formatter(config, env_name):
    """Use this for torchvision.transforms in multi-task dataset,
    note eval_fn always feeds in traj['obs']['images'], i.e. shape (h,w,3)
    """

    def resize_crop(img, bb=None, agent=False):
        """applies to every timestep's RGB obs['camera_front_image']"""
        task_spec = config.tasks_cfgs.get(env_name, dict())
        img_height, img_width = img.shape[:2]
        """applies to every timestep's RGB obs['camera_front_image']"""
        if len(getattr(task_spec, "demo_crop", OrderedDict())) != 0 and not agent:
            crop_params = task_spec.get(
                "demo_crop", [0, 0, 0, 0])
        if len(getattr(task_spec, "agent_crop", OrderedDict())) != 0 and agent:
            crop_params = task_spec.get(
                "agent_crop", [0, 0, 0, 0])
        if len(getattr(task_spec, "task_crops", OrderedDict())) != 0:
            crop_params = task_spec.get(
                "task_crops", [0, 0, 0, 0])
        if len(getattr(task_spec, "crop", OrderedDict())) != 0:
            crop_params = task_spec.get(
                "crop", [0, 0, 0, 0])

        top, left = crop_params[0], crop_params[2]
        img_height, img_width = img.shape[0], img.shape[1]
        box_h, box_w = img_height - top - \
            crop_params[1], img_width - left - crop_params[3]

        img = transforms.ToTensor()(img.copy())
        # ---- Resized crop ----#
        img = resized_crop(img, top=top, left=left, height=box_h,
                            width=box_w, size=(config.dataset_cfg.height, config.dataset_cfg.width))
        return img
        
    return resize_crop